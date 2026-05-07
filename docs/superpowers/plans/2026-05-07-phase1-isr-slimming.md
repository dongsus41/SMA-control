# Phase 1: ISR 슬림화 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** TIM4 / TIM5 ISR 본체를 메인 루프로 옮기고 ISR은 `tick++` 1줄만 남긴다. 의미 변경 없음 — 변경 전후 동작이 외부에서 동일해야 한다.

**Architecture:** ISR이 `g_sys.fast_tick` / `g_sys.slow_tick` 카운터를 atomic 증가시키고, 메인 루프(`while(1)`)가 `__disable_irq()` 짧은 critical section으로 atomic-swap한 뒤 동일한 본체 코드를 cooperative하게 호출한다. 본체는 `do_fast_tick()` / `do_slow_tick()` static 함수로 추출.

**Tech Stack:** STM32G474 (Cortex-M4), STM32CubeIDE, HAL 라이브러리. 펌웨어 단위 테스트 인프라 없음 — 변경 검증은 on-target manual verification (UART 로그 + PC GUI 응답).

**참조 문서:** `docs/superpowers/specs/2026-05-07-isr-pipeline-refactor-design.md` 섹션 1, 6 (Phase 1)

---

## 사전 검증 — 변경 전 baseline 기록

리팩토링이 동작 동일성을 깨지 않았음을 확인하려면 변경 전 baseline이 필요합니다.

- [ ] **Step 0.1: 현재 펌웨어 빌드 + 플래시**

STM32CubeIDE에서 현재 main 브랜치 코드를 빌드하여 보드에 플래시.

- [ ] **Step 0.2: Baseline 동작 기록**

PC에서 `cd SMA_control && python main.py` 실행. 다음 시나리오를 한 번씩 수행하고 결과를 메모:

1. **수동 PWM**: 채널 0에 PWM 50% 명령 → GUI에 PWM 50%가 표시되고 SMA가 가열되는지
2. **PID 온도 제어**: 채널 0 PID 활성화, 목표 50 °C → 그래프상 온도 응답 (overshoot, settling time)
3. **안전 모드**: 온도가 80 °C 초과 시 fan 강제 ON 동작 (드라이어 등으로 일시 가열)
4. **UART 송신율**: GUI 그래프 갱신 주기 체감 (대략적)

이 baseline을 후속 task의 검증 기준으로 사용. 메모는 commit 메시지나 별도 메모 파일로.

---

## Task 1: SystemState_t에 tick / overrun 카운터 필드 추가

기존 `g_sys` 구조체는 `ctrl_tick` 단일 카운터 + `ctrl_overrun_*` 등을 가지고 있는데, 두 박자(fast/slow)로 분리되므로 필드를 확장합니다. 기존 `ctrl_*` 필드는 이번 phase에서는 손대지 않고 새 필드만 추가 (하위 호환).

**Files:**
- Modify: `Core/Inc/system_defs.h` (SystemState_t 구조체 확장)

- [ ] **Step 1.1: `SystemState_t`에 fast/slow 필드 추가**

`Core/Inc/system_defs.h`의 `SystemState_t` 구조체 정의를 다음으로 변경:

```c
typedef struct
{
    volatile uint32_t ctrl_tick;            /**< (legacy) TIM ISR 증가 — 본 phase에서 미사용, 후속 정리 */
    uint32_t          ctrl_overrun_count;   /**< (legacy) */
    uint32_t          ctrl_overrun_max;     /**< (legacy) */
    uint32_t          ctrl_loop_hz;         /**< (legacy) */
    uint32_t          ctrl_loop_period_ms;  /**< (legacy) */
    uint8_t           state_command;        /**< (legacy) */

    /* ── 두 박자 cooperative 처리용 (Phase 1 신규) ──
     * fast_tick : TIM4 ISR(~250Hz) 증가, main loop atomic-swap 리셋
     * slow_tick : TIM5 ISR(~10Hz)  증가, main loop atomic-swap 리셋
     * volatile 필수 — ISR과 main loop 교차 접근.
     * `if (>0) { =0; }` 2줄 RMW는 race 가능 → __disable_irq() 짧은 critical section.
     *
     * overrun_count / overrun_max : main loop가 한 주기 이상 걸려 여러 틱이
     * 쌓인 경우 (pending-1)을 누적, 관측된 최대 pending을 기록. catch-up
     * 실행은 하지 않음(PID dt 왜곡 / I2C burst 회피). 정상 상태에선 count=0,
     * max≤1. main loop 단독 접근 → volatile 불필요.
     */
    volatile uint32_t fast_tick;
    volatile uint32_t slow_tick;
    uint32_t          fast_overrun_count;
    uint32_t          fast_overrun_max;
    uint32_t          slow_overrun_count;
    uint32_t          slow_overrun_max;
} SystemState_t;
```

- [ ] **Step 1.2: 빌드 확인**

STM32CubeIDE에서 빌드. 에러 없이 컴파일되어야 함.

Expected: 0 errors, 0 warnings (또는 기존 warnings 동일 수). BSS 0 초기화로 새 필드는 자동 0으로 시작하므로 별도 init 코드 불필요.

- [ ] **Step 1.3: Commit**

```bash
git add Core/Inc/system_defs.h
git commit -m "refactor(phase1): SystemState_t에 fast/slow tick 카운터 추가

ISR 슬림화 준비 — 두 박자 cooperative 처리용 카운터.
기존 ctrl_* 필드는 미사용 상태로 둠 (후속 phase에서 정리)."
```

---

## Task 2: TIM4 ISR 슬림화 + fast_tick 메인 루프 처리

현재 TIM4 ISR이 6채널 SPI 스캔, 센서 유효성 검사, UART 상태 송신을 직접 수행. 이걸 `do_fast_tick()` 함수로 추출하고 ISR은 `g_sys.fast_tick++` 1줄만 남깁니다. 메인 루프에서 atomic-swap 후 호출.

**Files:**
- Modify: `Core/Src/main.c` (HAL_TIM_PeriodElapsedCallback의 TIM4 분기, while(1), 새 static 함수)

- [ ] **Step 2.1: `do_fast_tick()` static 함수 추가**

`Core/Src/main.c`의 `HAL_TIM_PeriodElapsedCallback` 정의 **앞**(USER CODE BEGIN 0 영역 안, 173번 줄 직전)에 다음 함수 추가:

```c
/* ═══════════════ TIM4 (fast tick, ~250Hz) 본체 ═══════════════
 * Phase 1: 기존 TIM4 ISR 본체 그대로 main loop로 이동.
 * 의미 변경 없음 — 호출 컨텍스트만 ISR → main loop.
 */
static void do_fast_tick(void)
{
    TMC_Scan(CTRL_CH);

    for (uint8_t i = 0; i < CTRL_CH; i++)
    {
        pid.shared_data.temp_data[i] = tmc.temp_ext14[i];

        // 센서 유효성 즉시 검사
        Check_Temperature_Sensor(i, pid.shared_data.temp_data[i]);

        system.buf_fdcan_tx.struc.fan[i]  = system.state_fsw[i];
        system.buf_fdcan_tx.struc.pwm[i]  = *system.pnt_pwm[i];
        system.buf_fdcan_tx.struc.temp[i] = tmc.temp_ext14_raw[i];
    }

    // 타임스탬프 및 플래그 설정
    pid.shared_data.temp_timestamp = HAL_GetTick();
    pid.shared_data.new_temp_data  = 1;

    UartComm_SendState();

    //★ 힘 제어 모드일 때 힘/변위 상태도 전송
    if (force_ctrl.mode == CTRL_MODE_FORCE)
    {
        UartComm_SendForceState();
    }
}
```

- [ ] **Step 2.2: TIM4 ISR 분기를 `fast_tick++`로 교체**

`HAL_TIM_PeriodElapsedCallback` 함수 내 TIM4 분기(현재 175~203줄)를 다음으로 교체:

```c
	if(htim->Instance == htim4.Instance)
	{
		g_sys.fast_tick++;
	}
```

(주의: `else if(htim->Instance == htim5.Instance)` 블록은 Task 3에서 처리하므로 이번 task에서는 그대로 둡니다.)

- [ ] **Step 2.3: while(1)에 fast_tick 처리 로직 추가**

`main()` 함수의 `while (1)` 블록(현재 375~380줄, 비어있음)을 다음으로 교체:

```c
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/* ── fast_tick 처리 (~250Hz) ──
		 * atomic swap — 3 instruction 수준의 짧은 critical section.
		 * pending=1은 정상, 2 이상부터가 main loop 지연.
		 * catch-up 없이 1회만 실행 (SPI/UART burst 회피).
		 */
		if (g_sys.fast_tick > 0U)
		{
			uint32_t pending;

			__disable_irq();
			pending           = g_sys.fast_tick;
			g_sys.fast_tick   = 0U;
			__enable_irq();

			if (pending > 1U)
			{
				g_sys.fast_overrun_count += (pending - 1U);
				if (pending > g_sys.fast_overrun_max)
				{
					g_sys.fast_overrun_max = pending;
				}
			}

			do_fast_tick();
		}
	}
  /* USER CODE END 3 */
```

- [ ] **Step 2.4: 빌드 확인**

STM32CubeIDE에서 빌드. 컴파일 에러 / warning 없는지 확인.

Expected: 0 errors. `do_fast_tick`이 static으로 선언됐고 한 곳에서만 호출되므로 컴파일러가 인라인 가능성 (성능 영향 미미).

- [ ] **Step 2.5: 플래시 + 동작 검증 (사용자 수행)**

보드에 플래시 후 PC GUI 실행하여 Step 0.2 baseline과 비교:

1. **그래프 갱신** — 온도 그래프가 baseline과 동일한 속도로 갱신되는지. (UART 송신은 fast_tick에서 발생 → 250Hz 주기 유지되어야 함)
2. **수동 PWM** — 채널 0에 50% 명령 → GUI에 즉시 반영되고 SMA 가열 동일.
3. **온도값 표시** — 6채널 온도가 baseline과 비슷한 값/변동 폭.
4. **`pid.shared_data.new_temp_data` flag** — TIM5 PID(다음 task) 가 이 flag를 사용. 본 task 후 PID 응답이 baseline과 동일해야 정상.

**문제 발견 시 대처:**
- 그래프 갱신이 끊김 → main loop 다른 작업이 fast_tick 처리를 막고 있는지 확인. 현재 while(1)에 다른 작업 없으므로 발생 가능성 낮음.
- `g_sys.fast_overrun_count` 가 증가 → main loop 한 주기가 4ms 초과. UartComm_SendState()의 HAL_UART_Transmit timeout(10ms)이 의심.

- [ ] **Step 2.6: Commit**

```bash
git add Core/Src/main.c
git commit -m "refactor(phase1): TIM4 ISR을 tick++로 슬림화

기존 ISR 본체(SPI 스캔 + UART 송신)를 do_fast_tick() static 함수로
이동. main loop가 atomic-swap 후 cooperative 호출. 의미 변경 없음.

검증: GUI 그래프 갱신율, PWM 명령, 온도 표시 모두 baseline 동일."
```

---

## Task 3: TIM5 ISR 슬림화 + slow_tick 메인 루프 처리

TIM5 ISR(PID + 힘제어 + 팬 제어)도 동일 패턴으로 분리. Task 2와 같은 기법이지만 본체가 더 큼 (startup_phase 카운터, 힘제어/온도제어 분기 포함).

**Files:**
- Modify: `Core/Src/main.c` (HAL_TIM_PeriodElapsedCallback의 TIM5 분기, while(1), 새 static 함수)

- [ ] **Step 3.1: `do_slow_tick()` static 함수 추가**

`Core/Src/main.c`의 `do_fast_tick()` 정의 **바로 다음**에 추가:

```c
/* ═══════════════ TIM5 (slow tick, ~10Hz) 본체 ═══════════════
 * Phase 1: 기존 TIM5 ISR 본체 그대로 main loop로 이동.
 * 의미 변경 없음.
 *
 * startup_counter는 함수 내 static으로 보존 — 기존 ISR 내 static과 동일 동작.
 */
static void do_slow_tick(void)
{
    static uint8_t startup_counter = 0;
    if (pid.startup_phase) {
        startup_counter++;
        if (startup_counter >= 10) {  // 10번의 slow_tick 후 (약 1초)
            pid.startup_phase = 0;
            startup_counter = 0;
        }
    }

    for (uint8_t i = 0; i < CTRL_CH; i++)
    {
        // ★ 힘 제어
        if (force_ctrl.mode == CTRL_MODE_FORCE && force_ctrl.force_pid.enabled && i == force_ctrl.active_channel)
        {
            // I2C로 로드셀 데이터 읽기
            LoadCell_Read(&hi2c2, &force_ctrl.loadcell);

            if (force_ctrl.loadcell.valid)
            {
                float ctrl_output = ForceControl_Calculate(force_ctrl.loadcell.force);
                Set_PWM_Output(i, (uint8_t)ctrl_output);
            }
            else
            {
                // 센서 에러 시 안전 장치
                Set_PWM_Output(i, 0);
            }
            continue; // 이 채널은 온도제어 건너뜀
        }

        // ★ 온도제어
        // 1. 안전 온도 확인 (최우선 처리)
        bool in_safety_mode = Check_Safety_Temperature(i, pid.shared_data.temp_data[i]);

        // 2. PID 활성화 상태 및 안전 모드 확인
        if (pid.enable_pid[i] && !in_safety_mode)
        {
            float current_temp = pid.shared_data.temp_data[i];
            float target_temp  = pid.params[i].setpoint;

            // 센서 이상 확인
            // bool sensor_ok = Check_Temperature_Rise_Rate(i, current_temp);
            bool sensor_ok = true; // 온도 상승 안전모드 일단 주석처리

            if (sensor_ok)
            {
                // 제어 연산 수행
                float ctrl_output = Calculate_Ctrl(&pid.params[i], current_temp, i);
                Set_PWM_Output(i, (uint8_t)ctrl_output);

                // 온도 기반 팬 제어
                Control_Fan_By_Temperature(i, current_temp, target_temp);
            }
        }
    }
}
```

- [ ] **Step 3.2: TIM5 ISR 분기를 `slow_tick++`로 교체**

`HAL_TIM_PeriodElapsedCallback`의 TIM5 분기(Task 2 적용 후 현재 ~178줄부터의 `else if` 블록)를 다음으로 교체:

```c
	else if(htim->Instance == htim5.Instance)
	{
		g_sys.slow_tick++;
	}
```

이로써 `HAL_TIM_PeriodElapsedCallback` 전체가 다음과 같이 단순해집니다:

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim4.Instance)
	{
		g_sys.fast_tick++;
	}
	else if(htim->Instance == htim5.Instance)
	{
		g_sys.slow_tick++;
	}
}
```

- [ ] **Step 3.3: while(1)에 slow_tick 처리 로직 추가**

Task 2에서 추가한 `while (1)` 블록의 `do_fast_tick()` 호출 직후, 닫는 중괄호 **앞**에 slow_tick 처리를 추가:

```c
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/* ── fast_tick 처리 (~250Hz) ── */
		if (g_sys.fast_tick > 0U)
		{
			uint32_t pending;

			__disable_irq();
			pending           = g_sys.fast_tick;
			g_sys.fast_tick   = 0U;
			__enable_irq();

			if (pending > 1U)
			{
				g_sys.fast_overrun_count += (pending - 1U);
				if (pending > g_sys.fast_overrun_max)
				{
					g_sys.fast_overrun_max = pending;
				}
			}

			do_fast_tick();
		}

		/* ── slow_tick 처리 (~10Hz) ── */
		if (g_sys.slow_tick > 0U)
		{
			uint32_t pending;

			__disable_irq();
			pending          = g_sys.slow_tick;
			g_sys.slow_tick  = 0U;
			__enable_irq();

			if (pending > 1U)
			{
				g_sys.slow_overrun_count += (pending - 1U);
				if (pending > g_sys.slow_overrun_max)
				{
					g_sys.slow_overrun_max = pending;
				}
			}

			do_slow_tick();
		}
	}
  /* USER CODE END 3 */
```

- [ ] **Step 3.4: 빌드 확인**

STM32CubeIDE에서 빌드. 컴파일 에러 / warning 없는지 확인.

Expected: 0 errors. `do_slow_tick`이 `bool` 타입을 사용하므로 `<stdbool.h>` include가 main.c에 (또는 main.h를 통해) 이미 있는지 확인 — 현재 main.h:48에 `#include <stdbool.h>` 있어 OK.

- [ ] **Step 3.5: 플래시 + 동작 검증 (사용자 수행)**

보드에 플래시 후 PC GUI 실행하여 Step 0.2 baseline과 비교:

1. **PID 응답** — 채널 0 PID 활성화, 목표 50 °C → 그래프상 응답이 baseline과 거의 동일한지 (overshoot, settling time, steady-state error). PID 박자 = slow_tick = ~10Hz로 동일.
2. **수동 PWM 모드** — PID 비활성 상태에서 PWM 명령 → 정상 적용되고 변경 안 됨.
3. **안전 모드** — 드라이어로 온도 80 °C 초과 → fan 강제 ON, 회복 시 정상 모드 복귀 (히스테리시스 동일 동작).
4. **힘 제어** (구성돼 있다면) — `diag.py` 또는 GUI Force 모드 → force 응답 동일.
5. **`pid.startup_phase`** — 부팅 후 약 1초간 PID 비활성, 이후 활성 (10번의 slow_tick 후) — 시작 시 SMA 갑자기 가열되지 않는지 확인.
6. **Overrun 카운터** — 디버거로 `g_sys.fast_overrun_count`, `g_sys.slow_overrun_count`가 0 또는 매우 낮은 값을 유지하는지. 증가하면 main loop 한 주기가 박자보다 길다는 뜻.

**문제 발견 시 대처:**
- PID 응답이 baseline과 다름 → `do_slow_tick()` 안 본체가 ISR 안과 동일한지 line-by-line 비교. `static uint8_t startup_counter` 위치 (함수 내 static) 확인.
- `Check_Safety_Temperature` 또는 다른 함수가 ISR 컨텍스트 가정을 하고 있는지 (HAL_GetTick() 등은 main 컨텍스트에서도 동작).

- [ ] **Step 3.6: Commit**

```bash
git add Core/Src/main.c
git commit -m "refactor(phase1): TIM5 ISR을 tick++로 슬림화

기존 ISR 본체(PID + 힘제어 + 팬 제어)를 do_slow_tick() static 함수로
이동. HAL_TIM_PeriodElapsedCallback이 두 ISR 모두 tick++ 1줄로 정리.
의미 변경 없음.

검증: PID 응답, 수동 PWM, 안전 모드, startup_phase 모두 baseline 동일."
```

---

## 완료 후 상태

Phase 1 완료 시:
- `HAL_TIM_PeriodElapsedCallback` ≤ 10줄
- 모든 SPI / I2C / UART 호출이 main loop에서 발생 (ISR 안 X)
- `g_sys.fast_overrun_count` / `slow_overrun_count`로 main loop 지연 가시화 가능
- main.c는 여전히 큼 (모듈 추출은 Phase 3) — 이번 phase의 가시적 변화는 ISR 정리에만 한정

다음 phase: **Phase 2 — UART RX 분리**. 별도 plan 문서로 작성 예정.

---

## 자체 검토 결과

**Spec coverage**: 디자인 문서 섹션 6 Phase 1 ("TIM4/TIM5 ISR을 `tick++`만 남기고, 기존 본체를 main.c의 `do_fast_tick()`/`do_slow_tick()`으로 이동. while(1)에서 atomic-swap 후 호출.") — Task 1, 2, 3가 정확히 이 범위를 커버.

**Placeholder scan**: TBD/TODO 없음. 모든 코드 블록이 완전한 상태.

**Type consistency**: `g_sys.fast_tick`, `g_sys.slow_tick`, `g_sys.fast_overrun_count`, `g_sys.fast_overrun_max`, `g_sys.slow_overrun_count`, `g_sys.slow_overrun_max` — 모든 task에서 동일한 이름 사용. `do_fast_tick()` / `do_slow_tick()` 함수명도 일관.

**범위 / 안전성**: 각 task가 독립적으로 빌드 가능 (Task 1 단독 빌드 OK, Task 2 단독 빌드 OK — TIM5 ISR은 그대로). Task 2 검증 통과 후에만 Task 3 진행 → 회귀 시 의심 범위 좁음.
