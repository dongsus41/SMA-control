# Phase 5: Loadcell Ring Buffer + 12ms-cap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 로드셀을 fast_tick(~250Hz) 처리에서 12 ms-cap (≈83Hz) 폴링하여 8샘플 ring buffer에 누적, slow_tick(10Hz) Force PID는 ring buffer 평균값을 사용. 노이즈 감쇠 + force 응답 분해능 향상.

**Architecture:** `loadcell_i2c.c`에 module-internal ring buffer (8 floats) + 폴링 시간 캡 추가. 신규 API `LoadCell_Update / GetAverage / GetLatestDisp / IsStale`. 기존 `LoadCell_Read`는 그대로 유지 (CMD 0x06 I2C Test에서 사용 중). `Sensor_Update`가 fast_tick 박자로 `LoadCell_Update(hi2c, HAL_GetTick())` 호출 — 내부에서 12ms 캡 검사 후 read + push. `Controller_Update`의 CH_FORCE 분기는 `LoadCell_Read` 직접 호출 대신 `LoadCell_GetAverage()` 사용.

**Tech Stack:** STM32G474 + HAL I2C, Arduino+HX711 슬레이브 (80Hz 갱신, 12.5ms 주기).

**참조 문서:**
- `docs/superpowers/specs/2026-05-07-isr-pipeline-refactor-design.md` 섹션 5 (로드셀 슬레이브 인터페이스), 섹션 6 Phase 5

---

## File Structure

| 파일 | 변경 내용 |
|---|---|
| `Core/Inc/loadcell_i2c.h` | API 4개 추가: `LoadCell_Update`, `LoadCell_GetAverage`, `LoadCell_GetLatestDisp`, `LoadCell_IsStale` |
| `Core/Src/loadcell_i2c.c` | module-internal ring buffer + 12ms-cap 폴링 + 평균 함수 4개 |
| `Core/Src/controller.c` | `Sensor_Update`에서 `LoadCell_Update` 호출, `Controller_Update` CH_FORCE 분기에서 평균값 사용 |

3 파일 변경, 단일 task.

---

## Task 1: Loadcell ring buffer + 평균 사용

**Files:**
- Modify: `Core/Inc/loadcell_i2c.h`
- Modify: `Core/Src/loadcell_i2c.c`
- Modify: `Core/Src/controller.c`

- [ ] **Step 1.1: `loadcell_i2c.h`에 신규 API declaration 추가**

`Core/Inc/loadcell_i2c.h` 의 기존 declaration 다음에 추가:

```c
HAL_StatusTypeDef LoadCell_Read(I2C_HandleTypeDef *hi2c, LoadCell_Data_TypeDef *data);
void LoadCell_Init(LoadCell_Data_TypeDef *data);
```

다음으로 변경:

```c
HAL_StatusTypeDef LoadCell_Read(I2C_HandleTypeDef *hi2c, LoadCell_Data_TypeDef *data);
void LoadCell_Init(LoadCell_Data_TypeDef *data);

/* ── Phase 5: Ring buffer 평균 API ──
 * 슬레이브(Arduino+HX711) 80Hz 갱신 (12.5ms). fast_tick(~250Hz)에서
 * LoadCell_Update 호출하면 내부 12ms cap으로 ~83Hz 폴링 → 8샘플 ring
 * buffer 누적. Force PID(slow_tick 10Hz)는 LoadCell_GetAverage() 사용.
 *
 * 동기화: 단일 호출자(main loop fast_tick)만 update, force PID도 같은
 * main 컨텍스트에서 read — lock 불필요.
 */

/** @brief 12ms-cap 폴링 + ring buffer push.
 *  @param hi2c    I2C handle (hi2c2)
 *  @param now_ms  HAL_GetTick() 결과
 *  fast_tick 처리에서 매 호출. 12ms 미만 경과 시 즉시 return (캡).
 */
void LoadCell_Update(I2C_HandleTypeDef *hi2c, uint32_t now_ms);

/** @brief 8샘플 ring buffer의 force 평균값 반환 (g).
 *  buffer가 partial(8개 미만)이면 채워진 분량만 평균. 빈 상태면 0.0.
 */
float LoadCell_GetAverage(void);

/** @brief 최신 displacement 값 반환 (mm). 평균 안 함 (변위는 bias 거의 없음). */
float LoadCell_GetLatestDisp(void);

/** @brief 마지막 valid read 후 timeout_ms 경과했는지 검사.
 *  @return 0 = 신선, 1 = stale (Force PID는 stale 시 PWM=0 안전 차단)
 */
uint8_t LoadCell_IsStale(uint32_t now_ms, uint32_t timeout_ms);
```

- [ ] **Step 1.2: `loadcell_i2c.c`에 ring buffer 구현 추가**

`Core/Src/loadcell_i2c.c` 파일 끝에 다음 추가:

```c
/* ═══════════════ Phase 5: Ring Buffer 평균 ═══════════════ */

#define LOADCELL_AVG_SIZE     8U      /* 8샘플 평균 → 슬레이브 80Hz × 8 = ~10Hz force PID 박자 */
#define LOADCELL_POLL_CAP_MS  12U     /* HX711 80Hz (12.5ms)와 거의 동일 */

static float    s_force_buf[LOADCELL_AVG_SIZE];
static uint8_t  s_buf_idx     = 0;    /* 다음 write 위치 (0~7 wrap) */
static uint8_t  s_buf_filled  = 0;    /* 현재 채워진 샘플 수 (max 8) */
static float    s_latest_disp = 0.0f;
static uint32_t s_last_read_ms  = 0;  /* 마지막 시도(성공/실패 무관) */
static uint32_t s_last_valid_ms = 0;  /* 마지막 성공 read */

void LoadCell_Update(I2C_HandleTypeDef *hi2c, uint32_t now_ms)
{
    /* 12ms 캡 — 슬레이브 갱신 주기보다 빨리 폴링 안 함 */
    if ((uint32_t)(now_ms - s_last_read_ms) < LOADCELL_POLL_CAP_MS) return;
    s_last_read_ms = now_ms;

    LoadCell_Data_TypeDef data;
    LoadCell_Init(&data);
    if (LoadCell_Read(hi2c, &data) == HAL_OK && data.valid)
    {
        s_force_buf[s_buf_idx] = data.force;
        s_buf_idx = (uint8_t)((s_buf_idx + 1U) % LOADCELL_AVG_SIZE);
        if (s_buf_filled < LOADCELL_AVG_SIZE) s_buf_filled++;
        s_latest_disp   = data.displacement;
        s_last_valid_ms = now_ms;
    }
    /* 실패 시 ring buffer 변경 없음 — 이전 평균 유지 */
}

float LoadCell_GetAverage(void)
{
    if (s_buf_filled == 0U) return 0.0f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < s_buf_filled; i++) {
        sum += s_force_buf[i];
    }
    return sum / (float)s_buf_filled;
}

float LoadCell_GetLatestDisp(void)
{
    return s_latest_disp;
}

uint8_t LoadCell_IsStale(uint32_t now_ms, uint32_t timeout_ms)
{
    return ((uint32_t)(now_ms - s_last_valid_ms) > timeout_ms) ? 1U : 0U;
}
```

- [ ] **Step 1.3: `controller.c`의 `Sensor_Update`에서 `LoadCell_Update` 호출**

`Core/Src/controller.c`의 `Sensor_Update` 함수를 다음으로 교체:

```c
void Sensor_Update(void)
{
	TMC_Scan(CTRL_CH);

	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		g_state.temp[i] = tmc.temp_ext14_raw[i];
	}

	/* Phase 5: 로드셀 12ms-cap 폴링 + ring buffer 누적 */
	LoadCell_Update(&hi2c2, HAL_GetTick());
}
```

- [ ] **Step 1.4: `controller.c`의 `Controller_Update` CH_FORCE 분기 변경**

`Core/Src/controller.c`의 CH_FORCE case를 다음으로 교체:

```c
		case CH_FORCE:
		{
			/* Phase 5: ring buffer 8샘플 평균 사용. stale 검출 시 안전 차단. */
			if (LoadCell_IsStale(HAL_GetTick(), 100U))  /* 100ms 동안 새 데이터 없으면 stale */
			{
				g_ctrl.cmd_pwm[i] = 0;
				g_ctrl.cmd_fan[i] = 1;
			}
			else
			{
				float force_avg    = LoadCell_GetAverage();
				float target_force = (float)g_cmd.target[i] / 10.0f;  /* 0.1g/LSB */

				/* transitional: ForceControl_Calculate가 force_ctrl.force_pid 직접 참조 */
				force_ctrl.force_pid.target_force = target_force;
				force_ctrl.force_pid.enabled      = 1;

				float ctrl_output = ForceControl_Calculate(force_avg);
				g_ctrl.cmd_pwm[i] = (uint8_t)ctrl_output;
				g_ctrl.cmd_fan[i] = 0;
			}

			/* SendForceState 송신용 캐시 갱신 (transitional, Phase 6 정리 예정) */
			force_ctrl.loadcell.force        = LoadCell_GetAverage();
			force_ctrl.loadcell.displacement = LoadCell_GetLatestDisp();
			force_ctrl.loadcell.valid        = !LoadCell_IsStale(HAL_GetTick(), 100U);
			break;
		}
```

(주: 기존 `LoadCell_Data_TypeDef loadcell; LoadCell_Init(&loadcell); if (LoadCell_Read(...)) { ... }` 블록을 위 코드로 통째 교체. CH_FORCE 분기 진입 시 더 이상 직접 I2C read 안 함 — Sensor_Update가 이미 ring buffer에 갱신 중)

- [ ] **Step 1.5: 빌드 확인 (사용자 수행)**

다른 로컬에서 클론/풀 후 STM32CubeIDE 빌드. 0 errors. 신규 함수 4개 모두 declaration / definition 매칭 확인.

- [ ] **Step 1.6: 플래시 + 검증 (사용자 수행)**

| # | 시나리오 | 확인 포인트 |
|---|---|---|
| 1 | **Force 모드 진입** | `diag.py` 또는 GUI에서 force enable → CMD 0x05 force_state 송신, force/disp 표시 |
| 2 | **Force 응답 노이즈** | 정지 상태에서 force 그래프 — Phase 4 대비 노이즈 진폭 감소 (8샘플 평균 효과) |
| 3 | **Force PID 응답** | 목표 50g → 응답 시간이 baseline과 비슷 (slow_tick 10Hz 박자 동일) |
| 4 | **Force 모드 종료 후 재진입** | Off → Force → Off → Force 반복 시 매번 정상 응답 |
| 5 | **로드셀 분리 시** | 슬레이브 분리 (I2C 응답 없음) → 100ms 후 stale 검출 → PWM=0, fan=ON |
| 6 | **온도 제어 / 수동 모드** | Phase 4 대비 변경 없음 — baseline 동일 |

문제 발견 시:
- Force PID 출력이 0으로 고정 → ring buffer 채워지기 전(첫 8샘플 ~100ms) 평균이 정상인지 확인 (`s_buf_filled` 계산 로직). 부팅 직후 force 활성 시 ~100ms 지연 OK.
- Force 응답이 너무 느림 → 8샘플 평균이 group delay (~50ms) 추가. PID gain 재튜닝 필요할 수 있음. baseline과 비슷한 응답 원하면 buffer size를 4 또는 6으로 줄이는 옵션.

- [ ] **Step 1.7: Commit + push**

```bash
cd "C:\Users\Dongsu\Desktop\SMA-control"
git add Core/Inc/loadcell_i2c.h Core/Src/loadcell_i2c.c Core/Src/controller.c
git commit -m "$(cat <<'EOF'
refactor(phase5): 로드셀 ring buffer + 12ms-cap 폴링

fast_tick(~250Hz) 처리에서 LoadCell_Update 호출 — 내부 12ms 캡으로
~83Hz 폴링 → 8샘플 ring buffer 누적. slow_tick(10Hz) Force PID는
LoadCell_GetAverage() 사용. 노이즈 감쇠 + 분해능 향상.

- loadcell_i2c.h: LoadCell_Update / GetAverage / GetLatestDisp / IsStale 신규
- loadcell_i2c.c: module-internal ring buffer (8 floats) + cap + 평균
- controller.c Sensor_Update: LoadCell_Update 호출 추가
- controller.c CH_FORCE: LoadCell_Read 직접 호출 → ring buffer 평균 사용
   stale 검출 시 (100ms 무응답) 안전 차단

기존 LoadCell_Read는 유지 — CMD 0x06 I2C Test에서 사용 중.

검증: force 응답 노이즈 감소, baseline 응답 시간 유지.
EOF
)"
git push origin main
```

---

## 완료 후 상태

Phase 5 완료 시:
- 로드셀 폴링 박자가 fast_tick(~250Hz)으로 통합 — 슬레이브 80Hz 한계에 맞춰 12ms cap
- Force PID 입력에 8샘플 평균 적용 — 노이즈 감쇠 (group delay ~50ms 추가)
- Stale 검출 (100ms timeout) — 슬레이브 단선 시 자동 안전 차단
- `Controller_Update` CH_FORCE 분기에서 직접 I2C read 제거 → slow_tick 처리 시간 단축

다음 phase: **Phase 6 — 정리 + channel.h 흡수** (사용자가 우선순위로 언급한 통합 작업).

---

## 자체 검토 결과

**Spec coverage**:
- 디자인 spec 섹션 5 로드셀 슬레이브 인터페이스 — Step 1.2 (12ms cap이 슬레이브 12.5ms와 일치)
- 섹션 6 Phase 5 — Step 1.1~1.4가 정확히 커버

**Placeholder scan**: TBD/TODO 없음. 모든 코드 블록 완전.

**Type consistency**:
- `LoadCell_Update / GetAverage / GetLatestDisp / IsStale` — Step 1.1 declaration, Step 1.2 정의, Step 1.3/1.4 사용 모두 동일 시그니처
- `LOADCELL_AVG_SIZE = 8`, `LOADCELL_POLL_CAP_MS = 12` 일관 사용
- `s_force_buf / s_buf_idx / s_buf_filled / s_latest_disp / s_last_read_ms / s_last_valid_ms` static 변수명 일관

**범위 / 안전성**:
- 단일 task / 단일 commit. 검증 실패 시 `git revert` 한 번으로 롤백.
- 기존 `LoadCell_Read` 그대로 유지 — CMD 0x06 핸들러 영향 없음.
- 신규 변수가 모두 module-internal static — 외부 의존 없음.
- ring buffer 동기화: 단일 producer(Sensor_Update via fast_tick) + 단일 consumer(Controller_Update via slow_tick), 둘 다 main loop 컨텍스트라 race condition 없음.
- Phase 4 검증되지 않은 상태에서 추가 변경 — Phase 4 회귀 시 의심 범위 넓어지는 점 인지.
