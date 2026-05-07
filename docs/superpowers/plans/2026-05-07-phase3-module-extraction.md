# Phase 3: 모듈 추출 (controller.c) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** main.c의 `do_fast_tick / do_slow_tick` 본체를 신규 파일 `controller.c`의 `Sensor_Update / Controller_Update` 두 함수로 옮긴다. 의미 변경 없음 — 호출 순서·ordering 모두 보존.

**Architecture:** controller.c가 새 파일로 생기고 fast/slow 박자의 본체 로직을 담당. 기존 글로벌 (`system / pid / force_ctrl / tmc`)은 그대로 사용 (Phase 4에서 정식 분리 예정). main.c의 `while(1)` 루프는 `if (fast_tick > 0)` 안에서 직접 `Sensor_Update / UartComm_SendState / UartComm_SendForceState` 순서로, `if (slow_tick > 0)` 안에서 `Controller_Update` 직접 호출.

**Tech Stack:** STM32G474 / STM32CubeIDE. 신규 .c 파일은 일반적으로 IDE의 자동 source path scan(`Core/Src/*.c`)으로 빌드 포함됨 — 만약 자동 인식 안 되면 IDE에서 Project → Refresh 또는 source folder 재등록 필요.

**참조 문서:** `docs/superpowers/specs/2026-05-07-isr-pipeline-refactor-design.md` 섹션 2 (모듈 구조), 섹션 6 (Phase 3)

---

## Phase 3 범위 조정 (디자인 문서와 다른 점)

디자인 문서는 4함수(Sensor_Update / Safety_Update / Controller_Update / Actuator_Apply) 분리를 명시하지만, "의미 변경 없음" 원칙 하에서 완전 4분할은 데이터 모델 변경(중간 버퍼 `cmd_pwm[6]` 등)을 동반해야 한다.

예: 현재 do_fast_tick의 한 루프에서 `Check_Temperature_Sensor → buf_fdcan_tx.fan[i] = state_fsw[i]` 순서로 처리되는데, Sensor / Safety를 분리하면 `state_fsw` 갱신과 buf 쓰기 사이 ordering이 1 사이클 어긋나 의미 변경이 발생한다.

따라서 Phase 3에서는:
- **2함수만 분리** — `Sensor_Update` (do_fast_tick 본체 전체, 송신 제외) + `Controller_Update` (do_slow_tick 본체 전체)
- `Safety_Update / Actuator_Apply` 분리는 데이터 모델 변경과 함께 **Phase 4로 미룸**

Phase 3는 controller.c 신규 파일 도입 + 두 큰 함수의 *위치 이동*에 집중.

---

## File Structure

| 파일 | 변경 내용 |
|---|---|
| `Core/Inc/controller.h` | **신규**. `Sensor_Update()`, `Controller_Update()` declaration. |
| `Core/Src/controller.c` | **신규**. 두 함수 구현. 기존 do_fast_tick / do_slow_tick 본체 그대로. extern으로 `system`, `pid`, `force_ctrl`, `tmc`, `hi2c2` 참조. |
| `Core/Src/main.c` | `do_fast_tick` / `do_slow_tick` static 함수 제거. `while(1)`의 fast/slow tick 처리 블록에서 신규 함수 직접 호출. `controller.h` include 추가. |

신규 파일 두 개는 한 task에서 함께 생성 + main.c도 같은 task에서 수정해야 빌드 통과.

---

## Task 1: controller.c/h 신규 + Sensor_Update 분리

`do_fast_tick` 본체를 controller.c의 `Sensor_Update`로 이동. `do_slow_tick`은 이번 task에서 그대로 둠. 새 파일 생성 + main.c 수정 한 commit.

**Files:**
- Create: `Core/Inc/controller.h`
- Create: `Core/Src/controller.c`
- Modify: `Core/Src/main.c`

- [ ] **Step 1.1: `controller.h` 신규 파일 생성**

`Core/Inc/controller.h` 다음 내용으로 생성:

```c
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

/*
 * controller.c — 제어 파이프라인 단계별 함수 모음.
 *
 * Phase 3: do_fast_tick / do_slow_tick 본체를 main.c에서 분리.
 *   기존 글로벌(system / pid / force_ctrl / tmc) 그대로 사용.
 *   2함수 (Sensor_Update / Controller_Update)만 분리.
 *
 * Phase 4 예정: g_cmd / g_ctrl / g_state 데이터 모델 도입과 함께
 *   Safety_Update / Actuator_Apply 정식 분리.
 *
 * 호출 사이트: main.c의 while(1) 루프
 *   if (g_sys.fast_tick > 0) → Sensor_Update + UartComm_SendState + (force) UartComm_SendForceState
 *   if (g_sys.slow_tick > 0) → Controller_Update
 */

/** @brief fast_tick (~250Hz) 본체.
 *  - MAX31855 SPI 6채널 스캔
 *  - 채널별 센서 유효성 검사 (Check_Temperature_Sensor)
 *  - PC 송신 buffer (system.buf_fdcan_tx) 채우기
 *  - 타임스탬프 / new_temp_data flag 설정
 *  송신(UartComm_SendState 등)은 호출 사이트(main.c)에서 직접 수행.
 */
void Sensor_Update(void);

/** @brief slow_tick (~10Hz) 본체.
 *  - startup_phase 카운터 진행
 *  - 채널별 force / temp PID 계산 + Set_PWM_Output + 팬 제어
 */
void Controller_Update(void);

#endif /* __CONTROLLER_H */
```

- [ ] **Step 1.2: `controller.c` 신규 파일 생성 (Sensor_Update만)**

`Core/Src/controller.c` 다음 내용으로 생성:

```c
#include "controller.h"
#include "main.h"
#include "system_defs.h"
#include "max31855.h"
#include "temp_control.h"
#include "force_control.h"
#include "loadcell_i2c.h"

/* 기존 글로벌 참조 (Phase 4에서 정식 모델로 교체 예정) */
extern System_typedef            system;
extern PID_Manager_typedef       pid;
extern ForceControl_TypeDef      force_ctrl;
extern MAX31855_typedef          tmc;
extern I2C_HandleTypeDef         hi2c2;

/* ═══════════════ fast_tick (~250Hz) 본체 ═══════════════
 * Phase 3: do_fast_tick 본체 그대로 이동. 송신 호출은 호출 사이트로 분리.
 */
void Sensor_Update(void)
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
}

/* Controller_Update는 Task 2에서 추가 */
```

- [ ] **Step 1.3: `main.c`에서 `do_fast_tick` 제거 + Sensor_Update 직접 호출**

`Core/Src/main.c` Includes 영역(현재 24~27줄)에 controller.h 추가:

```c
#include "system_defs.h"
#include "uart_protocol.h"
#include "loadcell_i2c.h"
#include "force_control.h"
```

다음으로 변경:

```c
#include "system_defs.h"
#include "uart_protocol.h"
#include "loadcell_i2c.h"
#include "force_control.h"
#include "controller.h"
```

같은 파일에서 `do_fast_tick` static 함수 정의 (현재 `Manual_Control` 다음에 위치) 전체를 **삭제**:

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

(`do_slow_tick`은 그대로 유지 — Task 2에서 처리)

같은 파일에서 `while(1)`의 fast_tick 처리 블록(현재):

```c
		/* ── fast_tick 처리 (~250Hz) ──
		 * atomic swap — 3 instruction 수준의 짧은 critical section.
		 * pending=1은 정상, 2 이상부터가 main loop 지연.
		 * catch-up 없이 1회만 실행 (SPI/UART burst 회피).
		 */
		if (g_sys.fast_tick > 0U)
		{
			uint32_t pending;

			__disable_irq();
			pending          = g_sys.fast_tick;
			g_sys.fast_tick  = 0U;
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
```

다음으로 변경:

```c
		/* ── fast_tick 처리 (~250Hz) ──
		 * atomic swap — 3 instruction 수준의 짧은 critical section.
		 * pending=1은 정상, 2 이상부터가 main loop 지연.
		 * catch-up 없이 1회만 실행 (SPI/UART burst 회피).
		 */
		if (g_sys.fast_tick > 0U)
		{
			uint32_t pending;

			__disable_irq();
			pending          = g_sys.fast_tick;
			g_sys.fast_tick  = 0U;
			__enable_irq();

			if (pending > 1U)
			{
				g_sys.fast_overrun_count += (pending - 1U);
				if (pending > g_sys.fast_overrun_max)
				{
					g_sys.fast_overrun_max = pending;
				}
			}

			Sensor_Update();
			UartComm_SendState();
			if (force_ctrl.mode == CTRL_MODE_FORCE)
			{
				UartComm_SendForceState();
			}
		}
```

- [ ] **Step 1.4: 빌드 확인 (사용자 수행)**

다른 로컬에서 클론/풀 후 STM32CubeIDE 빌드. 확인 사항:

- 0 errors. controller.c가 자동 source path scan으로 빌드 포함되는지 확인 (Project Explorer에서 `Core/Src/controller.c`가 보이는지).
- 만약 controller.c가 빌드에 안 들어가서 link 에러 (`undefined reference to Sensor_Update`)가 나면:
  1. STM32CubeIDE에서 `Core` 폴더 우클릭 → `Refresh` (F5)
  2. 그래도 안 되면 Project → Properties → C/C++ General → Paths and Symbols → Source Location에 `Core/Src` 등록 확인
  3. Project → Clean → Build All

- [ ] **Step 1.5: 플래시 + 검증 (사용자 수행)**

보드에 플래시 후 다음 항목 확인:

1. **그래프 갱신율** — 6채널 온도 그래프 정상 갱신
2. **수동 PWM** — 채널 0에 50% → 즉시 반영
3. **PID 응답** — 목표 50 °C → 응답 동일 (TIM5 본체는 이번 task에서 변경 없음)
4. **안전 모드** — 80 °C 초과 시 fan 강제 ON (`Check_Temperature_Sensor`가 Sensor_Update 안에서 호출되므로 safety 동작 보존돼야 함)
5. **`g_sys.fast_overrun_count`** — 0 또는 매우 낮은 값

문제 발견 시:
- 온도 표시가 다름 → controller.c의 `extern` 글로벌이 정확히 매칭되는지 확인 (`system_typedef`, `PID_Manager_typedef`, `MAX31855_typedef` 정의된 헤더가 include 됐는지)
- buf_fdcan_tx에 0 또는 stale 값 → `system` 글로벌 참조가 main.c와 같은 인스턴스인지 (extern 선언이 controller.c에 정확히 있어야)

- [ ] **Step 1.6: Commit + push**

```bash
cd "C:\Users\Dongsu\Desktop\SMA-control"
git add Core/Inc/controller.h Core/Src/controller.c Core/Src/main.c
git commit -m "$(cat <<'EOF'
refactor(phase3): Sensor_Update를 controller.c로 분리

do_fast_tick 본체를 controller.c의 Sensor_Update()로 이동.
송신 호출(UartComm_SendState, UartComm_SendForceState)은
main.c의 while(1) 루프에서 직접 수행 — pipeline의 dispatcher 역할.
의미 변경 없음.

- controller.h / controller.c 신규
- 기존 글로벌(system / pid / force_ctrl / tmc) extern 참조
- main.c의 do_fast_tick static 함수 제거

검증: 그래프 갱신, 수동 PWM, PID 응답, 안전 모드 모두 baseline 동일.
EOF
)"
git push origin main
```

---

## Task 2: Controller_Update 분리

`do_slow_tick` 본체를 `controller.c`의 `Controller_Update`로 이동. main.c의 `do_slow_tick` static 제거 + 직접 호출.

**Files:**
- Modify: `Core/Src/controller.c` (Controller_Update 함수 추가)
- Modify: `Core/Src/main.c`

- [ ] **Step 2.1: `controller.c`에 Controller_Update 추가**

`Core/Src/controller.c` 의 `Sensor_Update` 함수 **다음**(`/* Controller_Update는 Task 2에서 추가 */` 주석 자리)에 추가:

```c
/* ═══════════════ slow_tick (~10Hz) 본체 ═══════════════
 * Phase 3: do_slow_tick 본체 그대로 이동.
 * startup_counter는 함수 내 static으로 보존 — 기존과 동일 동작.
 */
void Controller_Update(void)
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

(주: `bool` 타입은 `<stdbool.h>` 필요. main.h가 controller.c에서 include되고 main.h:48에 `<stdbool.h>` 있어 OK)

- [ ] **Step 2.2: `main.c`에서 `do_slow_tick` 제거 + Controller_Update 직접 호출**

`Core/Src/main.c` 의 `do_slow_tick` static 함수 정의 (현재 `do_fast_tick`이 제거되어 있는 상태에서 `Manual_Control` 다음 위치) 전체를 **삭제**:

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

같은 파일의 `while(1)`의 slow_tick 처리 블록 (현재):

```c
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
```

다음으로 변경:

```c
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

			Controller_Update();
		}
```

- [ ] **Step 2.3: 빌드 확인 (사용자 수행)**

다른 로컬에서 빌드. 0 errors 확인.

- [ ] **Step 2.4: 플래시 + 검증 (사용자 수행)**

보드에 플래시 후 다음 항목 확인:

1. **PID 응답** — 목표 50 °C → overshoot/settling time이 baseline과 동일
2. **수동 PWM** — PID 비활성 상태에서 명령 즉시 적용
3. **안전 모드** — 80 °C 초과 시 fan 강제 ON / 회복 시 정상 모드 복귀
4. **부팅 ~1초 startup_phase** — SMA가 부팅 직후 갑자기 가열되지 않는지 (`startup_counter`가 함수 내 static으로 보존됐는지)
5. **힘 제어** (구성됐다면) — diag.py force enable → 정상 응답
6. **`g_sys.slow_overrun_count`** — 0 유지

문제 발견 시:
- PID 응답이 다름 → controller.c의 Controller_Update 본체가 do_slow_tick과 line-by-line 동일한지 비교. `static uint8_t startup_counter` 위치(함수 내) 확인.
- force 제어가 안 됨 → `hi2c2`, `force_ctrl` extern이 controller.c에 정확히 선언됐는지

- [ ] **Step 2.5: Commit + push**

```bash
cd "C:\Users\Dongsu\Desktop\SMA-control"
git add Core/Src/controller.c Core/Src/main.c
git commit -m "$(cat <<'EOF'
refactor(phase3): Controller_Update를 controller.c로 분리

do_slow_tick 본체를 controller.c의 Controller_Update()로 이동.
PID + 힘제어 + 팬제어 로직이 모두 한 함수에. main.c의 while(1)에서
직접 호출. 의미 변경 없음.

Phase 3 완료 — main.c에서 do_fast_tick / do_slow_tick wrapper 제거,
controller.c가 fast/slow 박자 본체 담당.

Phase 4에서 데이터 모델(g_cmd / g_ctrl / g_state)과 함께
Safety_Update / Actuator_Apply 정식 분리 예정.

검증: PID 응답, 수동 PWM, 안전 모드, startup_phase, 힘 제어
모두 baseline 동일.
EOF
)"
git push origin main
```

---

## 완료 후 상태

Phase 3 완료 시:
- main.c의 ISR/제어 루프 관련 코드 ~70줄 감소 (do_fast_tick + do_slow_tick wrapper 제거)
- `Core/Src/controller.c` 새 모듈로 fast/slow 박자 본체 캡슐화
- main.c의 `while(1)`은 pipeline dispatcher 역할 — overrun 처리 + Sensor_Update / UartComm_Send* / Controller_Update / UartComm_Process 호출
- 의미 변경 없음 — PID 응답 / 안전 모드 / 송수신 모두 baseline 동일

다음 phase: **Phase 4 — 채널 모드 일반화 (D-1)**. 호환성 깨지는 큰 변경. 새 데이터 모델 (`g_cmd / g_ctrl / g_state`) 도입, CMD 0x01 페이로드 의미 재정의, CMD 0x04 폐기, PC 측 코드(`uart_driver.py`, `main.py`, `ctrl.py`, `diag.py`, `Force test.py`) 동시 변경. 별도 plan 문서로 작성 예정.

---

## 자체 검토 결과

**Spec coverage**: 디자인 문서 섹션 6 Phase 3 — 4함수 spec을 2함수로 축소(Phase 4와 책임 분담). 축소 근거를 plan 상단 "Phase 3 범위 조정"에 명시. Task 1 (Sensor_Update) + Task 2 (Controller_Update)가 본 plan의 범위를 커버.

**Placeholder scan**: TBD/TODO 없음. 모든 코드 블록이 완전한 상태. Step 1.2 끝의 `/* Controller_Update는 Task 2에서 추가 */` 주석은 placeholder가 아닌 task 진행 안내.

**Type consistency**:
- `Sensor_Update()` / `Controller_Update()` 시그니처 — Step 1.1 declaration, Step 1.2 정의, Step 1.3 / 2.2 호출 모두 동일
- extern 글로벌 (`system / pid / force_ctrl / tmc / hi2c2`) — Step 1.2에서 일괄 선언, Step 1.2 / 2.1 본체에서 사용
- `static uint8_t startup_counter` — Step 2.1에서 함수 내 static으로 정의. Phase 1에서 do_slow_tick에 있던 것과 동일 위치/동작

**범위 / 안전성**: 두 task가 독립 빌드/검증 가능 (Task 1만 적용 후 do_slow_tick은 그대로 → 빌드 OK, 동작 OK). 회귀 시 의심 범위 좁음. 신규 파일 두 개라 IDE source path scan 의존성 명시 (Step 1.4).
