# Phase 4: 채널 모드 일반화 (D-1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 채널별 `ctrl_mode` enum (`OFF / MANUAL / TEMP / FORCE`) 도입. 기존 `system.ctrl_param + pid.enable_pid + force_ctrl.mode` 비대칭 분기를 단일 모델로 통일. CMD 0x01 페이로드 의미 재정의(30B 동일 크기), CMD 0x04 폐기. 펌웨어/PC 동시 업데이트 필요한 호환성 깨짐 단계.

**Architecture:** 신규 헤더 `channel.h`에 `ch_ctrl_e` enum + `cmd_param_t / ctrl_state_t / state_buf_t` 3분할 구조체 정의 (디자인 spec 섹션 3 그대로). controller.c에 글로벌 `g_cmd / g_ctrl / g_state` 정의. 파이프라인 4함수 (Sensor_Update / Safety_Update / Controller_Update / Actuator_Apply)로 정식 분리, Phase 3에서 미룬 Safety/Actuator도 이번에 함께. PC측은 `uart_driver.py`의 `send_control_message`에 mode 인자 추가, `send_force_control_message` 폐기.

**Tech Stack:** STM32G474 / STM32CubeIDE (펌웨어), PyQt5 + pyserial (PC). 펌웨어/PC 동시 업데이트 — 한 commit으로 묶음.

**참조 문서:** `docs/superpowers/specs/2026-05-07-isr-pipeline-refactor-design.md` 섹션 3, 4, 6 (Phase 4)

---

## 호환성 깨짐 안내

이 plan은 **펌웨어와 PC 코드 양쪽이 동시에 새 페이로드 포맷을 사용**해야 동작합니다:

- 새 펌웨어 + 구 PC → CMD 0x01 페이로드를 펌웨어가 잘못 해석해서 전 채널 OFF 또는 임의 동작 가능
- 구 펌웨어 + 새 PC → 펌웨어가 mode 바이트를 PWM 값으로 인식하여 위험한 PWM 출력 가능

따라서 **반드시 펌웨어 / PC를 동시에 업그레이드**. 다른 동료가 작업 중인 경우(`Seojin`이 최근 `main.py / uart_driver.py` 수정), commit 직전에 `git pull` + 충돌 확인 필수. 충돌 시 본 plan의 변경이 우선 (Phase 4 호환성이 깨지므로 PC 코드를 새 모델에 맞춰 다시 정리).

---

## Phase 4 범위 / 비범위

**포함:**
- 데이터 모델 도입 (`channel.h` 신규)
- 4함수 정식 분리 (Phase 3에서 미룬 Safety/Actuator 포함)
- UART CMD 0x01 페이로드 의미 재정의
- CMD 0x04 폐기
- PC 코드 5 파일 동시 변경

**비범위 (Phase 6에서):**
- 기존 거대 글로벌 (`system / pid / force_ctrl`) **완전** 제거
- 본 plan에서는 새 모델(`g_cmd / g_ctrl / g_state`)이 *주* 데이터 흐름. 기존 글로벌은 잔여 멤버만 남김 (예: `system.state_level`, `system.pnt_pwm[]`은 그대로). `system.ctrl_param_now / save / buf_fdcan_tx`, `pid.enable_pid / params / shared_data`, `force_ctrl.mode / active_channel` 등은 새 모델로 대체 후 정리

---

## File Structure

| 파일 | 변경 내용 |
|---|---|
| `Core/Inc/channel.h` | **신규**. `ch_ctrl_e`, `cmd_param_t`, `ctrl_state_t`, `state_buf_t` 정의 + 글로벌 `extern` 선언 |
| `Core/Inc/controller.h` | `Safety_Update()`, `Actuator_Apply()` 추가 |
| `Core/Src/controller.c` | `g_cmd / g_ctrl / g_state` 정의. Sensor_Update / Controller_Update 변경, Safety_Update / Actuator_Apply 신규. 일부 기존 글로벌 사용 코드 새 모델 사용 |
| `Core/Src/uart_protocol.c` | CMD 0x01 새 페이로드 디코드, CMD 0x02 송신은 `g_state` 사용, CMD 0x03 게인 업데이트는 `g_ctrl.temp_params` 향함, CMD 0x04 핸들러 제거, CMD 0x05 송신은 `g_cmd.mode` 참조 |
| `Core/Inc/uart_protocol.h` | `CMD_FORCE_CONTROL` define 제거, `FORCE_CTRL_PAYLOAD_SIZE` 제거 |
| `Core/Src/main.c` | `while(1)` fast/slow 처리 블록에서 4함수 호출 순서 새로 정리 |
| `Core/Src/temp_control.c` | `pid.enable_pid` / `pid.params` 사용처를 `g_ctrl` 향하도록 변경 (PID 함수의 인자로 받거나) |
| `Core/Inc/temp_control.h` | safety 함수 시그니처 정리 (필요 시) |
| `Core/Src/force_control.c` | `force_ctrl.mode / active_channel` 사용처 정리 |
| `Core/Inc/force_control.h` | mode/active_channel 멤버 제거 또는 deprecated 표시 |
| `SMA_control/uart_driver.py` | `send_control_message` 새 시그니처 (mode 인자 추가), `send_force_control_message` 제거, `CMD_FORCE_CONTROL` 상수 제거 |
| `SMA_control/main.py` | `apply_settings()` / `emergency_stop()`에서 새 `send_control_message` 호출, `send_force_control_message` 호출 모두 제거 |
| `SMA_control/ctrl.py` | 명령어를 새 페이로드에 맞게 변경 |
| `SMA_control/diag.py` | CMD 0x04 송신 제거, CMD 0x01 + mode=CH_FORCE로 대체 |
| `SMA_control/Force test.py` | 동일 |

---

## Task 1: Phase 4 통합 변경

펌웨어 + PC 동시 변경. 단일 commit. 21개 step (코드 영역별로 그룹화).

**Files:** (위 표 참조)

### A. 펌웨어 — 데이터 모델 도입

- [ ] **Step 1.1: `Core/Inc/channel.h` 신규 파일 생성**

`Core/Inc/channel.h` 다음 내용으로 생성:

```c
#ifndef __CHANNEL_H
#define __CHANNEL_H

#include <stdint.h>
#include "system_defs.h"      /* CTRL_CH */
#include "temp_control.h"     /* PID_Param_TypeDef */
#include "force_control.h"    /* Force_PID_TypeDef */

/*
 * channel.h — 채널 상태 데이터 모델 (Phase 4)
 *
 * 디자인 문서 섹션 3을 그대로 구현. 기존 거대 글로벌을 3분할:
 *   g_cmd   : PC 명령 입력
 *   g_ctrl  : controller 내부 상태 + 출력 (PID 파라미터, safety, cmd_pwm/fan)
 *   g_state : MCU → PC 송신 buffer
 *
 * Phase 6에서 system / pid / force_ctrl 잔여 멤버 정리 예정.
 */

/* ── 채널별 제어 모드 ── */
typedef enum {
    CH_OFF    = 0,   /* PWM=0, fan=off */
    CH_MANUAL = 1,   /* user-set PWM 직접 적용 */
    CH_TEMP   = 2,   /* 온도 PID */
    CH_FORCE  = 3,   /* 힘 PID (현재 단일 로드셀 — 동시 활성 1채널만) */
} ch_ctrl_e;

/* ── PC가 set, controller가 read (CMD 0x01 → 디코드) ──
 * 페이로드 매핑 (30B):
 *   mode[6] + manual_pwm[6] + manual_fan[6] + target[6 × u16]
 */
typedef struct {
    uint8_t  mode[CTRL_CH];         /* ch_ctrl_e */
    uint8_t  manual_pwm[CTRL_CH];
    uint8_t  manual_fan[CTRL_CH];   /* 0/1 */
    uint16_t target[CTRL_CH];       /* TEMP: 0.25°C/LSB, FORCE: 0.1g/LSB */
} cmd_param_t;

/* ── controller 내부 상태 (Sensor/Safety set, Controller produce, Actuator read) ── */
typedef struct {
    PID_Param_TypeDef   temp_params[CTRL_CH];   /* 온도 PID 파라미터 + 상태 */
    Force_PID_TypeDef   force_params[CTRL_CH];  /* 힘 PID 파라미터 + 상태 (현재 단일 채널만 사용) */
    uint8_t             safety_mode[CTRL_CH];   /* 0=normal / 1=warn(80°C+) / 2=critical(120°C+) / 3=sensor_err */
    uint8_t             cmd_pwm[CTRL_CH];       /* controller → actuator (0~100) */
    uint8_t             cmd_fan[CTRL_CH];       /* controller → actuator (0/1) */
} ctrl_state_t;

/* ── MCU → PC 송신 buffer (CMD 0x02 페이로드, 24B) ──
 * 기존 Buf_FDCAN_Tx_typedef와 동일 레이아웃. fan에 safety 코드 매립:
 *   0 = 정상 OFF, 1 = 정상 ON, 17 = warn, 49 = sensor_err
 */
typedef struct {
    uint8_t  pwm[CTRL_CH];
    uint8_t  fan[CTRL_CH];
    uint16_t temp[CTRL_CH];
} state_buf_t;

extern cmd_param_t   g_cmd;
extern ctrl_state_t  g_ctrl;
extern state_buf_t   g_state;

#endif /* __CHANNEL_H */
```

- [ ] **Step 1.2: `controller.c`에 글로벌 정의 + extern 정리**

`Core/Src/controller.c`의 include 영역과 글로벌 영역을 다음으로 변경:

```c
#include "controller.h"
#include "channel.h"
#include "main.h"
#include "system_defs.h"
#include "max31855.h"
#include "temp_control.h"
#include "force_control.h"
#include "loadcell_i2c.h"

/* ── Phase 4 신규 글로벌 ── */
cmd_param_t   g_cmd;
ctrl_state_t  g_ctrl;
state_buf_t   g_state;

/* 기존 글로벌 (Phase 6에서 정리 예정) — pnt_pwm은 actuator에서 사용 */
extern System_typedef            system;
extern MAX31855_typedef          tmc;
extern I2C_HandleTypeDef         hi2c2;
```

(주: `pid` / `force_ctrl` extern은 더 이상 controller.c에서 사용 안 함 — `g_ctrl.temp_params / force_params`로 대체됨. 사용처 정리 후 제거)

### B. 펌웨어 — 파이프라인 함수

- [ ] **Step 1.3: `controller.h`에 4함수 declaration 정리**

`Core/Inc/controller.h` 전체를 다음으로 교체:

```c
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

/*
 * controller.c — 제어 파이프라인 단계별 함수 (Phase 4 정식 4분할).
 *
 * 호출 사이트: main.c의 while(1) 루프
 *   if (g_sys.fast_tick > 0) {
 *       Sensor_Update();       // SPI 6채널 thermo + g_state.temp[] 채움
 *       Safety_Update();       // g_ctrl.safety_mode[] 갱신
 *       UartComm_SendState();
 *       if (force 활성 채널 있음) UartComm_SendForceState();
 *   }
 *   if (g_sys.slow_tick > 0) {
 *       Controller_Update();   // mode dispatch → g_ctrl.cmd_pwm/cmd_fan
 *       Actuator_Apply();      // safety overlay + PWM CCR + FSW GPIO + g_state.pwm/fan
 *   }
 */

void Sensor_Update(void);
void Safety_Update(void);
void Controller_Update(void);
void Actuator_Apply(void);

#endif /* __CONTROLLER_H */
```

- [ ] **Step 1.4: Sensor_Update 변경 — `g_state.temp[]` 사용**

`Core/Src/controller.c`의 `Sensor_Update` 함수 본체를 다음으로 교체:

```c
/* ═══════════════ fast_tick 본체 — Sensor 단계 ═══════════════
 * Phase 4: g_state.temp[]에 raw 측정값 저장. safety check는 Safety_Update로 분리.
 * temp_data는 controller가 직접 tmc.temp_ext14를 read하므로 별도 저장 불필요.
 */
void Sensor_Update(void)
{
	TMC_Scan(CTRL_CH);

	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		g_state.temp[i] = tmc.temp_ext14_raw[i];
	}
}
```

(주: 기존 `pid.shared_data.temp_data[i]`, `pid.shared_data.temp_timestamp`, `new_temp_data` flag, `system.buf_fdcan_tx`는 새 모델에서 더 이상 채우지 않음. Safety/Controller가 `tmc.temp_ext14`와 `g_state.temp`를 직접 사용. Actuator가 `g_state.pwm/fan`을 채움)

- [ ] **Step 1.5: Safety_Update 신규**

`Core/Src/controller.c`의 `Sensor_Update` 정의 다음에 추가:

```c
/* ═══════════════ fast_tick 본체 — Safety 단계 ═══════════════
 * 채널별 온도 임계 검사 + 히스테리시스. g_ctrl.safety_mode[i] 갱신.
 * controller는 safety_mode를 모름 — actuator overlay에서 강제 적용.
 *
 * 기존 Check_Temperature_Sensor + Check_Safety_Temperature 로직을 통합.
 * 임계값: warn=80°C, critical=120°C, sensor_err=>200°C or <-50°C
 * 히스테리시스: warn→normal=75°C 미만, critical→normal=30°C 이하
 */
void Safety_Update(void)
{
	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		float t = tmc.temp_ext14[i];
		uint8_t mode = g_ctrl.safety_mode[i];

		if (t > 200.0f || t < -50.0f) {
			mode = 3;  /* sensor error */
		}
		else if (mode == 3) {
			/* 센서 정상 복귀 시 mode 0으로 */
			mode = 0;
		}

		if (mode != 3) {
			if (t >= 120.0f) {
				mode = 2;
			}
			else if (mode == 2 && t <= 30.0f) {
				mode = 0;
			}
			else if (mode != 2) {
				if (t >= 80.0f) {
					mode = 1;
				}
				else if (mode == 1 && t < 75.0f) {
					mode = 0;
				}
			}
		}

		g_ctrl.safety_mode[i] = mode;
	}
}
```

- [ ] **Step 1.6: Controller_Update 변경 — mode dispatch + cmd produce**

`Core/Src/controller.c`의 `Controller_Update` 함수 본체를 다음으로 교체:

```c
/* ═══════════════ slow_tick 본체 — Controller 단계 ═══════════════
 * 채널별 g_cmd.mode 디스패치 → g_ctrl.cmd_pwm/cmd_fan 채움.
 * safety 무지 — actuator가 overlay 적용.
 *
 * Force 모드는 단일 로드셀 제약상 동시 1채널만 (UART 디코더에서 검증).
 * startup_phase는 첫 ~1초간 PID 비활성 — 부팅 직후 SMA 가열 방지.
 */
void Controller_Update(void)
{
	static uint8_t startup_counter = 0;
	static uint8_t startup_phase   = 1;
	if (startup_phase) {
		startup_counter++;
		if (startup_counter >= 10) {  /* 약 1초 */
			startup_phase   = 0;
			startup_counter = 0;
		}
	}

	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		switch ((ch_ctrl_e)g_cmd.mode[i])
		{
		case CH_OFF:
			g_ctrl.cmd_pwm[i] = 0;
			g_ctrl.cmd_fan[i] = 0;
			break;

		case CH_MANUAL:
			g_ctrl.cmd_pwm[i] = g_cmd.manual_pwm[i];
			g_ctrl.cmd_fan[i] = g_cmd.manual_fan[i];
			break;

		case CH_TEMP:
			if (startup_phase) {
				g_ctrl.cmd_pwm[i] = 0;
				g_ctrl.cmd_fan[i] = 0;
			} else {
				float current = tmc.temp_ext14[i];
				float target  = (float)g_cmd.target[i] / 4.0f;  /* 0.25°C/LSB */
				g_ctrl.temp_params[i].setpoint = target;

				float ctrl_output = Calculate_Ctrl(&g_ctrl.temp_params[i], current, i);
				g_ctrl.cmd_pwm[i] = (uint8_t)ctrl_output;

				/* 온도 기반 팬 결정 (현재 - 목표 차이로) */
				float diff = current - target;
				g_ctrl.cmd_fan[i] = (diff > 8.0f) ? 1 : (diff < 5.0f) ? 0 : g_ctrl.cmd_fan[i];
			}
			break;

		case CH_FORCE:
			/* 단일 로드셀 — UART 디코더가 1채널 보장.
			 * I2C로 로드셀 데이터 읽기 + force PID. */
			LoadCell_Data_TypeDef loadcell;
			LoadCell_Init(&loadcell);
			if (LoadCell_Read(&hi2c2, &loadcell) == HAL_OK && loadcell.valid) {
				float target_force = (float)g_cmd.target[i] / 10.0f;  /* 0.1g/LSB */
				g_ctrl.force_params[i].target_force = target_force;
				float ctrl_output = ForceControl_Calculate(loadcell.force);
				g_ctrl.cmd_pwm[i] = (uint8_t)ctrl_output;
				g_ctrl.cmd_fan[i] = 0;
			} else {
				g_ctrl.cmd_pwm[i] = 0;  /* 센서 에러 시 */
				g_ctrl.cmd_fan[i] = 1;  /* 안전 fan */
			}
			break;
		}
	}
}
```

(주: 기존 `Control_Fan_By_Temperature`의 히스테리시스 fan 로직을 inline으로 대체. 본 plan에서 fan 결정 로직은 단순화 — 목표와의 차이 8°C 초과면 ON, 5°C 미만이면 OFF, 그 사이는 유지. 기존 함수의 자세한 동작과 다를 수 있음 — 검증 시 baseline 비교 필요. 차이 발견 시 기존 로직 그대로 옮기도록 plan 갱신)

- [ ] **Step 1.7: Actuator_Apply 신규**

`Core/Src/controller.c`의 `Controller_Update` 정의 다음에 추가:

```c
/* ═══════════════ slow_tick 본체 — Actuator 단계 ═══════════════
 * controller가 produce한 cmd_pwm/cmd_fan에 safety overlay 적용.
 * PWM CCR + FSW GPIO 출력 + g_state.pwm/fan에 송신용 값 mirror.
 *
 * Safety overlay (한 곳에서만 강제):
 *   safety_mode 1 (warn)        → fan 강제 ON
 *   safety_mode 2 (critical) /
 *   safety_mode 3 (sensor_err)  → PWM=0, fan 강제 ON
 *
 * fan 송신 코드 매립 (기존 패턴 유지):
 *   0  : 정상 OFF
 *   1  : 정상 ON
 *   17 : warn
 *   49 : sensor_err
 */
void Actuator_Apply(void)
{
	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		uint8_t pwm  = g_ctrl.cmd_pwm[i];
		uint8_t fan  = g_ctrl.cmd_fan[i];
		uint8_t fan_out_code;  /* g_state.fan[i] 송신용 */

		switch (g_ctrl.safety_mode[i])
		{
		case 0:  /* normal */
			fan_out_code = fan ? 1 : 0;
			break;
		case 1:  /* warn */
			fan = 1;
			fan_out_code = 17;
			break;
		case 2:  /* critical */
			pwm = 0;
			fan = 1;
			fan_out_code = 17;  /* critical도 17로 표시 (기존 호환) */
			break;
		case 3:  /* sensor_err */
		default:
			pwm = 0;
			fan = 1;
			fan_out_code = 49;
			break;
		}

		/* 하드웨어 출력 */
		*system.pnt_pwm[i] = pwm;
		if (fan) FSW_on(i);
		else     FSW_off(i);

		/* 송신용 g_state */
		g_state.pwm[i] = pwm;
		g_state.fan[i] = fan_out_code;
	}
}
```

(주: `system.pnt_pwm[]`은 PWM CCR 매핑이라 그대로 사용. `FSW_on/off` 매크로도 그대로. 이 두 가지는 Phase 6에서 actuator 내부로 옮길 수 있지만 본 phase 범위 밖)

### C. 펌웨어 — UART 프로토콜

- [ ] **Step 1.8: `uart_protocol.c`의 CMD 0x01 디코더 변경**

`Core/Src/uart_protocol.c`의 include 영역에 channel.h 추가, extern 정리:

```c
#include "uart_protocol.h"
#include <stdio.h>
#include "loadcell_i2c.h"
#include "force_control.h"
#include "channel.h"

/* ... 기존 Private Variables / ring buffer 그대로 ... */

extern System_typedef       system;     /* state_level 용 */
extern cmd_param_t          g_cmd;
extern ctrl_state_t         g_ctrl;

/* pid, hi2c2 extern은 제거 — 새 모델에서 사용 안 함 */
```

`UartComm_HandleControl` 함수를 다음으로 교체:

```c
/*******************************************************************************
 * Handle Control Command (PC -> MCU) — CMD 0x01, 30 byte
 *   Layout (Phase 4 새 페이로드):
 *     mode[6]        : ch_ctrl_e (offset 0)
 *     manual_pwm[6]  : 0~100 (offset 6)
 *     manual_fan[6]  : 0/1 (offset 12)
 *     target[6×u16]  : TEMP 0.25°C/LSB / FORCE 0.1g/LSB (offset 18, little-endian)
 *
 *   FORCE 동시 1채널 제약: mode[]에 CH_FORCE가 2개 이상이면 프레임 거부.
 ******************************************************************************/
static void UartComm_HandleControl(const uint8_t *data, uint8_t len)
{
    if (len != CTRL_PAYLOAD_SIZE)  /* 30 */
        return;

    if (system.state_level != SYSTEM_GO)
        return;

    /* FORCE 동시 1채널 제약 검증 */
    uint8_t force_count = 0;
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        if (data[i] == CH_FORCE) force_count++;
    }
    if (force_count > 1) {
        printf("WARN: CMD_CONTROL rejected — multiple CH_FORCE channels (%u)\r\n", force_count);
        return;
    }

    /* mode 변경 시 PID 상태 reset */
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        if (g_cmd.mode[i] != data[i]) {
            g_ctrl.temp_params[i].u_old      = 0.0f;
            g_ctrl.temp_params[i].last_error = 0.0f;
            g_ctrl.safety_mode[i]            = 0;
        }
    }

    /* 페이로드 디코드 → g_cmd */
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        g_cmd.mode[i]       = data[0  + i];
        g_cmd.manual_pwm[i] = data[6  + i];
        g_cmd.manual_fan[i] = data[12 + i];
        g_cmd.target[i]     = (uint16_t)data[18 + i*2] | ((uint16_t)data[19 + i*2] << 8);
    }

    LED1_toggle;
}
```

- [ ] **Step 1.9: CMD 0x02 송신 변경 (`UartComm_SendState`)**

`Core/Src/uart_protocol.c`의 `UartComm_SendState`를 다음으로 교체:

```c
/*******************************************************************************
 * Send State Report (MCU -> PC) — CMD 0x02, 24 byte
 *   Phase 4: g_state buffer 사용 (Actuator_Apply가 채움).
 ******************************************************************************/
void UartComm_SendState(void)
{
    UartComm_SendFrame(CMD_STATE,
                       (const uint8_t*)&g_state,
                       STATE_PAYLOAD_SIZE);
}
```

(주: `state_buf_t`의 메모리 레이아웃 `pwm[6]+fan[6]+temp[6×u16]`은 기존 `Buf_FDCAN_Tx_typedef`와 동일하므로 raw cast 가능. uint16 little-endian은 ARM Cortex-M4 native로 OK)

- [ ] **Step 1.10: CMD 0x03 게인 업데이트 변경**

`Core/Src/uart_protocol.c`의 `UartComm_HandleGainUpdate`를 다음으로 교체:

```c
/*******************************************************************************
 * Handle Gain Update (PC -> MCU) — CMD 0x03, 13 byte
 *   Phase 4: g_ctrl.temp_params 향함.
 ******************************************************************************/
static void UartComm_HandleGainUpdate(const uint8_t *data, uint8_t len)
{
    if (len != GAIN_PAYLOAD_SIZE)
        return;

    float kp, ki, kd;
    memcpy(&kp, &data[0], 4);
    memcpy(&ki, &data[4], 4);
    memcpy(&kd, &data[8], 4);
    uint8_t ch = data[12];

    if (ch >= CTRL_CH) {
        printf("Error: Invalid channel number %u\r\n", ch);
        return;
    }

    /* PID_Param_TypeDef는 lambda/alpha/gain으로 저장 */
    g_ctrl.temp_params[ch].lambda = kp;
    g_ctrl.temp_params[ch].alpha  = ki;
    g_ctrl.temp_params[ch].gain   = kd;

    printf("PID Gains updated for CH %u: Kp=%.2f, Ki=%.4f, Kd=%.4f\r\n",
           ch, kp, ki, kd);
    LED2_toggle;
}
```

- [ ] **Step 1.11: CMD 0x04 (Force Control) 핸들러 폐기**

`Core/Src/uart_protocol.c`의 `UartComm_HandleForceControl` 함수 전체와 `UartComm_HandleFrame` 의 `case CMD_FORCE_CONTROL` 분기를 **삭제**:

함수 정의 삭제:

```c
static void UartComm_HandleForceControl(const uint8_t *data, uint8_t len)
{
    /* ... 전체 본체 ... */
}
```

`UartComm_HandleFrame` 의 분기에서 다음 부분 삭제:

```c
    case CMD_FORCE_CONTROL:
        UartComm_HandleForceControl(data, len);
        break;
```

`Core/Inc/uart_protocol.h`에서 `CMD_FORCE_CONTROL` define 및 관련 size 상수 삭제:

```c
#define CMD_FORCE_CONTROL     0X04   /* 삭제 */
#define FORCE_CTRL_PAYLOAD_SIZE    6  /* 삭제 */
```

(`CMD_FORCE_STATE` (0x05)와 `FORCE_STATE_PAYLOAD_SIZE` (10)는 유지 — MCU→PC 송신은 그대로 사용)

- [ ] **Step 1.12: CMD 0x05 (Force State) 송신 변경**

`Core/Src/uart_protocol.c`의 `UartComm_SendForceState`를 다음으로 교체:

```c
/*******************************************************************************
 * Send Force State Report (MCU -> PC) — CMD 0x05, 10 byte
 *   Phase 4: g_cmd.mode 참조하여 active force 채널 식별.
 *   동시 1채널 제약상 첫 CH_FORCE 채널만 송신.
 *   Layout: mode(1) + channel(1) + force(float32_LE) + displacement(float32_LE)
 ******************************************************************************/
void UartComm_SendForceState(void)
{
    /* 첫 force 채널 찾기 */
    int8_t force_ch = -1;
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        if (g_cmd.mode[i] == CH_FORCE) {
            force_ch = (int8_t)i;
            break;
        }
    }
    if (force_ch < 0) return;  /* force 채널 없음 */

    /* I2C 로드셀 read — Controller_Update가 이미 read한 시점이면 stale 가능,
     * 본 plan에서는 단순화 위해 송신 시 별도 read는 안 함. force_ctrl 잔여 멤버
     * 사용은 Phase 6 정리 시점에 별도 cache 필드로 옮길 예정. */
    uint8_t payload[FORCE_STATE_PAYLOAD_SIZE];
    payload[0] = CH_FORCE;
    payload[1] = (uint8_t)force_ch;
    /* force / displacement는 force_ctrl.loadcell의 잔여 캐시 사용 (transitional) */
    memcpy(&payload[2], (void*)&force_ctrl.loadcell.force, 4);
    memcpy(&payload[6], (void*)&force_ctrl.loadcell.displacement, 4);
    UartComm_SendFrame(CMD_FORCE_STATE, payload, FORCE_STATE_PAYLOAD_SIZE);
}
```

(주: `force_ctrl.loadcell` 캐시 사용은 transitional. Phase 6에서 별도 캐시 필드(`g_ctrl.force_loadcell` 등)로 옮길 예정. force_ctrl extern 선언이 필요하면 본 파일에 추가)

`Core/Src/uart_protocol.c`의 extern 영역에 추가:

```c
extern ForceControl_TypeDef force_ctrl;
```

- [ ] **Step 1.13: CMD 0x06 (I2C Test) 핸들러 유지**

기존 `UartComm_HandleI2CTest` 그대로. hi2c2 extern 선언 필요하면 본 파일에 유지.

```c
extern I2C_HandleTypeDef hi2c2;  /* I2C Test에서 사용 */
```

### D. 펌웨어 — main.c

- [ ] **Step 1.14: `while(1)` fast/slow 처리 블록 변경**

`Core/Src/main.c`의 `while(1)` 안의 두 블록을 다음으로 교체:

```c
		/* ── fast_tick 처리 (~250Hz) ── */
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
			Safety_Update();
			UartComm_SendState();

			/* Force 활성 채널 있으면 force state도 송신 */
			for (uint8_t i = 0; i < CTRL_CH; i++) {
				if (g_cmd.mode[i] == CH_FORCE) {
					UartComm_SendForceState();
					break;
				}
			}
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

			Controller_Update();
			Actuator_Apply();
		}
```

(`UartComm_Process()` 호출은 그대로 유지)

`Core/Src/main.c`의 include 영역에 channel.h 추가:

```c
#include "controller.h"
#include "channel.h"
```

### E. 펌웨어 — 잔여 빌드 에러 정리

- [ ] **Step 1.15: 빌드 후 발생한 에러 추적 + 잔여 사용처 정리**

이 시점에 빌드하면 기존 `system.ctrl_param_now`, `pid.enable_pid`, `pid.shared_data.temp_data`, `force_ctrl.mode`, `force_ctrl.active_channel`, `Manual_Control()`, `Set_PWM_Output()` 등의 사용처에서 빌드 에러 발생 가능. 다음 패턴으로 정리:

`Core/Src/main.c`의 `Manual_Control` 함수 — 이제 호출처 없으므로 **삭제**:

```c
/* Manual_Control — 더 이상 사용 안 함 (UART 디코더가 g_cmd 채우면 끝) */
void Manual_Control (uint8_t ch) { /* ... 전체 함수 본체 ... */ }
```

`Core/Inc/main.h`의 `Manual_Control` declaration **삭제**:

```c
void Manual_Control(uint8_t ch);  /* 삭제 */
```

`Core/Src/temp_control.c`에서 `pid.enable_pid` / `pid.shared_data` / `system.state_pwm` / `system.state_fsw` 등 사용처를 검토. PID 함수(`Calculate_Ctrl`)는 이미 인자 받는 형태이므로 호출 사이트만 변경됨 (controller.c에서 이미 `&g_ctrl.temp_params[i]` 전달). `Set_PWM_Output`, `Update_Fan_Status`, `Control_Fan_By_Temperature` 등 직접 출력 함수는 더 이상 호출되지 않으므로 함수 정의를 그대로 두되 사용처 없는 dead code 상태. **이번 phase에서는 그대로 유지** (Phase 6에서 정리).

`Core/Src/force_control.c`의 `force_ctrl.mode / active_channel / force_pid.enabled` 사용처 — `ForceControl_Calculate`는 단순히 force 값 받아 PWM 반환하면 OK. `ForceControl_SetTarget / Enable / Disable`은 사용처(CMD 0x04 핸들러)가 사라졌으므로 dead code. 본 phase에서 **그대로 유지** (Phase 6).

빌드 에러가 남으면 case-by-case로 추적: 각 에러의 사용처를 새 모델에 맞게 변경 (예: `system.ctrl_param_now.pwm[i]` → `g_cmd.manual_pwm[i]`).

### F. PC — uart_driver.py

- [ ] **Step 1.16: `uart_driver.py` 새 시그니처 + CMD 0x04 제거**

`SMA_control/uart_driver.py`의 다음 변경:

상수 영역에서 `CMD_FORCE_CONTROL = 0x04` 라인은 그대로 유지 (제거 가능하나 다른 모듈에서 import할 가능성 — 확인 후 판단):

```python
# CMD_FORCE_CONTROL = 0x04   # Phase 4: 폐기 (CMD 0x01 mode=CH_FORCE로 통합)
```

mode enum 추가 (파일 상단, 상수 영역):

```python
# Channel control modes (CMD 0x01 mode[6])
CH_OFF    = 0
CH_MANUAL = 1
CH_TEMP   = 2
CH_FORCE  = 3

# 표시 채널 — main.py에서 import (DISPLAY_CH는 main.py 자체 정의이므로 여기서 0 default)
DEFAULT_DISPLAY_CH = 0
```

`send_control_message` 메서드를 다음으로 교체:

```python
    def send_control_message(self, mode, manual_pwm, manual_fan, target,
                             display_ch=DEFAULT_DISPLAY_CH):
        """CMD 0x01 — Phase 4 새 페이로드 (30B).

        단일 표시 채널 운용 가정: display_ch만 mode/manual_pwm/manual_fan/target
        적용, 나머지 채널은 CH_OFF.

        Args:
            mode (int): CH_OFF / CH_MANUAL / CH_TEMP / CH_FORCE
            manual_pwm (int): 0~100 (CH_MANUAL일 때만 의미)
            manual_fan (bool): True/False (CH_MANUAL일 때만 의미)
            target (float): TEMP=°C, FORCE=g (mode 따라 LSB 변환)
            display_ch (int): 명령을 적용할 채널 (0~5)
        """
        if not self.ser or not self.ser.is_open:
            return

        # mode[6] / manual_pwm[6] / manual_fan[6]
        mode_arr       = [CH_OFF] * CTRL_CH
        manual_pwm_arr = [0]      * CTRL_CH
        manual_fan_arr = [0]      * CTRL_CH

        ch = max(0, min(CTRL_CH - 1, int(display_ch)))
        mode_arr[ch]       = int(mode)
        manual_pwm_arr[ch] = max(0, min(100, int(manual_pwm)))
        manual_fan_arr[ch] = 1 if manual_fan else 0

        # target[6 × u16] — display_ch만 raw 값, 나머지 0
        target_raw = [0] * CTRL_CH
        if mode == CH_TEMP:
            target_raw[ch] = max(0, min(0xFFFF, int(round(target * 4))))   # 0.25°C/LSB
        elif mode == CH_FORCE:
            target_raw[ch] = max(0, min(0xFFFF, int(round(target * 10))))  # 0.1g/LSB

        payload = bytearray()
        payload.extend(mode_arr)
        payload.extend(manual_pwm_arr)
        payload.extend(manual_fan_arr)
        for v in target_raw:
            payload.append(v & 0xFF)
            payload.append((v >> 8) & 0xFF)

        try:
            self.ser.write(build_frame(CMD_CONTROL, bytes(payload)))
        except serial.SerialException:
            pass
```

`send_force_control_message` 메서드 **전체 삭제**:

```python
    def send_force_control_message(self, channel, enable, target_force):
        """CMD 0x04 — 힘 제어 모드 설정"""
        # ... 전체 본체 — 삭제 ...
```

### G. PC — main.py

- [ ] **Step 1.17: `main.py` `apply_settings()` / `send_heartbeat()` / `emergency_stop()` 변경**

`SMA_control/main.py`의 import 영역에 채널 모드 상수 추가:

```python
from uart_driver import UartWorker, CH_OFF, CH_MANUAL, CH_TEMP, CH_FORCE
```

`apply_settings` 메서드를 다음으로 교체:

```python
    def apply_settings(self):
        """ctrl_mode → 채널 모드 매핑 후 send_control_message 1회 호출."""
        if self.ctrl_mode == MODE_MANUAL:
            self.current_pwm      = self.spin_pwm.value()
            self.current_fan      = self.chk_fan.isChecked()
            self.current_pid_mode = False
            self.current_target   = 0.0
            print(f"Applied: Manual  PWM={self.current_pwm}  FAN={self.current_fan}")

        elif self.ctrl_mode == MODE_TEMP:
            self.current_target   = self.spin_target.value()
            self.current_pid_mode = True
            print(f"Applied: Temp PID  Target={self.current_target}°C")

        else:  # MODE_FORCE
            self.current_force_target = self.spin_target_force.value()
            self.current_pid_mode     = False
            self.current_pwm          = 0
            print(f"Applied: Force Ctrl  Target={self.current_force_target}g  CH={DISPLAY_CH}")

        # 즉시 한 번 보내기 (heartbeat 대기 안 함)
        self._send_current_command()

    def _send_current_command(self):
        """현재 ctrl_mode + 입력값으로 CMD 0x01 1회 송신."""
        if self.ctrl_mode == MODE_MANUAL:
            self.worker.send_control_message(
                mode=CH_MANUAL,
                manual_pwm=self.current_pwm,
                manual_fan=self.current_fan,
                target=0.0,
                display_ch=DISPLAY_CH)
        elif self.ctrl_mode == MODE_TEMP:
            self.worker.send_control_message(
                mode=CH_TEMP,
                manual_pwm=0,
                manual_fan=False,
                target=self.current_target,
                display_ch=DISPLAY_CH)
        else:  # MODE_FORCE
            self.worker.send_control_message(
                mode=CH_FORCE,
                manual_pwm=0,
                manual_fan=False,
                target=self.current_force_target,
                display_ch=DISPLAY_CH)
```

`send_heartbeat` 메서드를 다음으로 교체:

```python
    def send_heartbeat(self):
        """100ms 주기로 현재 명령 재송신 (Force 모드 포함 — 모드별 분기 불필요)."""
        self._send_current_command()
```

`emergency_stop` 메서드의 송신 부분을 다음으로 교체:

```python
    def emergency_stop(self):
        print("!!! EMERGENCY STOP !!!")
        self.current_pwm      = 0
        self.current_fan      = False
        self.current_pid_mode = False
        self.current_target   = 0.0

        # 모든 모드 OFF
        self.worker.send_control_message(
            mode=CH_OFF,
            manual_pwm=0,
            manual_fan=False,
            target=0.0,
            display_ch=DISPLAY_CH)

        self.tx_timer.stop()
        self.plot_timer.stop()

        for sig, slot in [(self.worker.data_received,  self.handle_new_data),
                          (self.worker.force_received, self.handle_force_data)]:
            try:
                sig.disconnect(slot)
            except Exception:
                pass
```

### H. PC — 보조 스크립트

- [ ] **Step 1.18: `ctrl.py` 변경**

`SMA_control/ctrl.py`의 명령 디스패치 부분을 새 페이로드 사용하도록 변경. 핵심 변경: `m`/`t`/`f` 명령이 한 채널에만 적용하는 새 시그니처 `_send_control()` 헬퍼 도입.

`SMA_control/ctrl.py`에서 `CMD_FORCE_CONTROL = 0x04` 라인 다음에 mode enum 추가:

```python
# Channel control modes (Phase 4)
CH_OFF    = 0
CH_MANUAL = 1
CH_TEMP   = 2
CH_FORCE  = 3
```

기존 송신 함수들을 다음 헬퍼로 통합:

```python
def send_control(ser, mode, manual_pwm=0, manual_fan=False, target=0.0,
                 display_ch=DISPLAY_CH):
    """CMD 0x01 — Phase 4 새 페이로드 (30B)."""
    mode_arr       = [CH_OFF] * CTRL_CH
    manual_pwm_arr = [0]      * CTRL_CH
    manual_fan_arr = [0]      * CTRL_CH
    target_raw     = [0]      * CTRL_CH

    ch = max(0, min(CTRL_CH - 1, int(display_ch)))
    mode_arr[ch]       = int(mode)
    manual_pwm_arr[ch] = max(0, min(100, int(manual_pwm)))
    manual_fan_arr[ch] = 1 if manual_fan else 0
    if mode == CH_TEMP:
        target_raw[ch] = max(0, min(0xFFFF, int(round(target * 4))))
    elif mode == CH_FORCE:
        target_raw[ch] = max(0, min(0xFFFF, int(round(target * 10))))

    payload = bytearray()
    payload.extend(mode_arr)
    payload.extend(manual_pwm_arr)
    payload.extend(manual_fan_arr)
    for v in target_raw:
        payload.append(v & 0xFF)
        payload.append((v >> 8) & 0xFF)
    ser.write(build_frame(CMD_CONTROL, bytes(payload)))
```

ctrl.py의 명령 처리 부분에서 기존 `send_force_control_message` 호출 모두 `send_control(ser, CH_FORCE, target=N)`으로, 기존 manual/temp 송신은 `send_control(ser, CH_MANUAL/CH_TEMP, ...)` 패턴으로 변경. 사용처 grep 후 일괄 정리:

```bash
grep -n "CMD_FORCE_CONTROL\|build_frame(CMD_CONTROL\|build_frame(0x04\|build_frame(0x01" SMA_control/ctrl.py
```

각 호출 사이트를 새 헬퍼 호출로 교체. 변경량은 보통 5~10줄.

- [ ] **Step 1.19: `diag.py` 변경**

`SMA_control/diag.py`의 CMD 0x04 송신 코드를 CMD 0x01로 교체. 검색:

```bash
grep -n "CMD_FORCE_CONTROL\|0x04\|build_frame" SMA_control/diag.py
```

기존 `build_frame(CMD_FORCE_CONTROL, struct.pack('<BBf', ch, enable, target))` 패턴을 다음으로 교체:

```python
def force_control_frame(channel, enable, target_force):
    """CMD 0x01에 mode=CH_FORCE (or CH_OFF) 설정."""
    CH_OFF = 0
    CH_FORCE = 3
    mode_arr       = [CH_OFF] * 6
    manual_pwm_arr = [0]      * 6
    manual_fan_arr = [0]      * 6
    target_raw     = [0]      * 6

    if enable:
        mode_arr[channel]   = CH_FORCE
        target_raw[channel] = max(0, min(0xFFFF, int(round(target_force * 10))))

    payload = bytearray()
    payload.extend(mode_arr)
    payload.extend(manual_pwm_arr)
    payload.extend(manual_fan_arr)
    for v in target_raw:
        payload.append(v & 0xFF)
        payload.append((v >> 8) & 0xFF)
    return build_frame(0x01, bytes(payload))  # CMD_CONTROL
```

기존 `ser.write(build_frame(CMD_FORCE_CONTROL, ...))` 호출 사이트를 `ser.write(force_control_frame(FORCE_CHANNEL, True, FORCE_TARGET))` 등으로 교체.

`CMD_FORCE_CONTROL = 0x04` 상수는 유지하지 않음 (사용 안 함, 주석 처리 또는 제거).

- [ ] **Step 1.20: `Force test.py` 변경**

`SMA_control/Force test.py`도 동일 패턴. 검색 + 교체. (파일 크기 223줄이라 변경량 적음)

```bash
grep -n "CMD_FORCE_CONTROL\|0x04\|build_frame" SMA_control/'Force test.py'
```

각 호출을 위 Step 1.19의 `force_control_frame` 패턴으로 교체.

### I. 검증 + commit

- [ ] **Step 1.21: 빌드 + 플래시 + GUI 검증 (사용자 수행)**

다른 로컬에서 클론/풀 후:

1. **STM32CubeIDE 빌드** — `controller.c`, `channel.h` 모두 빌드에 포함되는지. dead code (Manual_Control, ForceControl_SetTarget 등) 관련 unused warning은 무시.
2. **PC Python 빌드** — `python main.py`, `python ctrl.py` 모두 import error 없는지. `from uart_driver import ...` 새 enum 인식.
3. **플래시 + GUI 실행**.

검증 항목:

| # | 시나리오 | 기대 동작 |
|---|---|---|
| 1 | Manual 모드 (DISPLAY_CH=0, PWM 50%, FAN ON) | 채널 0만 PWM 50%, FAN ON. 나머지 채널 OFF 유지 |
| 2 | Temp PID (목표 50 °C) | 채널 0 PID 응답 — overshoot/settling time이 baseline과 비슷 |
| 3 | Force 모드 (목표 50g) | `diag.py` 또는 GUI에서 force enable → 채널 0 force PID, force_state 송신 |
| 4 | Emergency Stop | 모든 채널 OFF 즉시 |
| 5 | 안전 모드 | 80 °C 초과 시 fan 강제 ON, 회복 시 정상 |
| 6 | PID Gain 업데이트 (CMD 0x03) | "PID Gains updated for CH 0..." 메시지 |
| 7 | 부팅 startup_phase | 부팅 직후 1초간 PID 비활성 |
| 8 | overrun 카운터 | `g_sys.fast_overrun_count`, `slow_overrun_count` 0 유지 |

특히 검증할 점:
- **CMD 0x01 mode=CH_FORCE** 가 정상 동작하는지 (구버전은 CMD 0x04 별도 송신 필요했음)
- **CH_OFF 모드** (모든 채널 OFF) 가 의도대로 PWM=0, fan=off 유지하는지

검증 실패 시 대처:
- 명령이 아예 안 받아짐 → `UartComm_HandleControl`의 `len != CTRL_PAYLOAD_SIZE` 체크 통과 여부 (`CTRL_PAYLOAD_SIZE`가 30인지 확인). 또는 PC `send_control_message`가 30B를 보내는지.
- TEMP 모드인데 setpoint 0°C로 시작 → `g_ctrl.temp_params[i].setpoint`가 새 mode 진입 시 갱신되는지. `g_ctrl.temp_params[i]` 가 BSS 0 초기화로 `lambda/alpha/gain` 모두 0인 경우 PID 출력 0 — 부팅 직후 게인 업데이트 (CMD 0x03) 필요할 수 있음.
- Force 응답 이상 → loadcell read는 Controller_Update에서 직접 호출. force_ctrl.loadcell 캐시 transitional 사용처 확인.

검증 통과하면 다음 step으로.

- [ ] **Step 1.22: Commit + push**

```bash
cd "C:\Users\Dongsu\Desktop\SMA-control"

# Seojin 동시 작업 가능성 — pull 한 번 더 (충돌 시 본 plan 변경 우선)
git fetch origin
git status

# 변경 파일 stage
git add Core/Inc/channel.h
git add Core/Inc/controller.h
git add Core/Inc/uart_protocol.h
git add Core/Src/controller.c
git add Core/Src/uart_protocol.c
git add Core/Src/main.c
git add Core/Inc/main.h
git add SMA_control/uart_driver.py
git add SMA_control/main.py
git add SMA_control/ctrl.py
git add SMA_control/diag.py
git add "SMA_control/Force test.py"

git commit -m "$(cat <<'EOF'
refactor(phase4): 채널 모드 일반화 (D-1) — 호환성 깨짐

채널별 ctrl_mode (OFF/MANUAL/TEMP/FORCE) 도입. 기존 시스템 전역
모드 + 채널별 enable_pid 비대칭 분기를 단일 모델로 통일.

펌웨어:
- channel.h 신규 — ch_ctrl_e, cmd_param_t, ctrl_state_t, state_buf_t
- g_cmd / g_ctrl / g_state 글로벌 (controller.c 정의)
- Safety_Update / Actuator_Apply 정식 분리 (Phase 3에서 미룬 분리)
- Sensor_Update / Controller_Update 변경 — 새 모델 사용
- main.c while(1) 호출 순서: Sensor → Safety → Send → Controller → Actuator
- UART CMD 0x01 페이로드 의미 재정의 (30B 동일 크기)
   mode[6] + manual_pwm[6] + manual_fan[6] + target[6×u16]
- UART CMD 0x04 (Force Control) 폐기 — CMD 0x01 mode=CH_FORCE로 통합
- Force 동시 1채널 제약 — UART 디코더에서 검증

PC:
- uart_driver.py — send_control_message 새 시그니처 (mode 인자),
   send_force_control_message 제거
- main.py — apply_settings / send_heartbeat / emergency_stop 새 모델
- ctrl.py / diag.py / Force test.py — CMD 0x01 통합 송신

호환성: 펌웨어/PC 동시 업그레이드 필수. 구 PC + 새 펌웨어,
또는 새 PC + 구 펌웨어 조합은 위험한 임의 동작 발생 가능.

기존 거대 글로벌 (system / pid / force_ctrl) 잔여 멤버는 dead code
상태로 유지 — Phase 6에서 정리 예정.

검증: Manual / Temp PID / Force / Emergency Stop / 안전 모드 모두
baseline과 비슷한 동작.
EOF
)"

git push origin main
```

---

## 완료 후 상태

Phase 4 완료 시:
- 채널별 `ctrl_mode` 모델 도입. 향후 다채널 동시 운용 가능 (force는 하드웨어 제약상 1채널).
- 4단계 단방향 파이프라인 (Sensor → Safety → Controller → Actuator) 명확.
- safety overlay 한 곳(`Actuator_Apply`)에서만 강제.
- UART 프로토콜 통합 — CMD 0x01 하나로 모든 모드 진입/해제.
- PC 코드 단순화 — `send_control_message` 단일 함수, mode 인자.
- 기존 거대 글로벌은 잔여 dead code. Phase 6 cleanup 대기.

다음 phase: **Phase 5 — loadcell ring buffer + 12ms-cap**. 별도 plan 문서.

---

## 자체 검토 결과

**Spec coverage**:
- 디자인 spec 섹션 3 (채널 상태 모델) — Step 1.1 (channel.h), Step 1.2 (글로벌)
- 섹션 4 (UART 프로토콜) — Step 1.8 (CMD 0x01), Step 1.9 (CMD 0x02), Step 1.10 (CMD 0x03), Step 1.11 (CMD 0x04 폐기), Step 1.12 (CMD 0x05)
- 섹션 5 (안전 / 에러 처리) — Step 1.5 (Safety_Update), Step 1.7 (Actuator_Apply overlay)
- 섹션 6 Phase 4 — Step 1.4~1.7 (4함수 분리), Step 1.8 (페이로드), PC 5 파일 (Step 1.16~1.20)

**Placeholder scan**: TBD/TODO 없음. 모든 코드 블록이 완전. 일부 PC 스크립트 변경(Step 1.18/1.19/1.20)은 호출 사이트가 적어 grep 후 case-by-case 교체 패턴 명시 — placeholder가 아니라 grep 명령 + 교체 코드 모두 제시.

**Type consistency**:
- `ch_ctrl_e` (CH_OFF=0, CH_MANUAL=1, CH_TEMP=2, CH_FORCE=3) — Step 1.1에서 정의, Step 1.6/1.8에서 펌웨어, Step 1.16~1.20에서 PC 모두 동일 값
- `cmd_param_t / ctrl_state_t / state_buf_t` 필드명 — Step 1.1 정의, Step 1.4~1.7 / 1.8~1.10 사용 일관
- 페이로드 매핑 (mode[6] offset 0, manual_pwm[6] offset 6, manual_fan[6] offset 12, target offset 18) — Step 1.1 (struct), Step 1.8 (펌웨어 디코더), Step 1.16/1.18 (PC 빌더) 모두 동일
- LSB 단위 (TEMP 0.25°C, FORCE 0.1g) — Step 1.6 (펌웨어 변환), Step 1.16/1.18/1.19 (PC 변환) 일치

**범위 / 안전성**:
- 단일 commit (펌웨어 + PC 동시) — incremental 빌드 가능 단계 명확:
  - Step 1.1~1.7 (펌웨어 데이터 모델 + 함수): 빌드 통과하지만 새 함수 미호출
  - Step 1.8~1.14 (UART + main.c): 펌웨어 빌드 + 호출 연결, 단 PC 변경 전이라 명령 송신 시 deserialize 실패
  - Step 1.16~1.20 (PC): 양쪽 호환 회복
- 호환성 깨짐이라 partial deploy 불가. 한 commit으로 묶음.
- 동료(Seojin) 작업 충돌 가능성 명시 (Step 1.22의 `git fetch / status`).
- 잔여 dead code (Manual_Control, ForceControl_SetTarget, Set_PWM_Output 등) Phase 6에서 정리 — 본 phase에서는 빌드 warning 무시.

**보완 사항**:
- Controller_Update의 fan 결정 로직 (Step 1.6)이 기존 `Control_Fan_By_Temperature`와 정확히 동일하지 않을 수 있음. 검증 단계(Step 1.21)에서 fan 동작 차이 발견 시 plan 갱신 + 기존 함수 그대로 사용 (`Control_Fan_By_Temperature(i, current, target)` 호출 후 그 결과를 `g_ctrl.cmd_fan[i]`에 mirror).
- Force state 송신(Step 1.12)에서 `force_ctrl.loadcell` transitional 사용 — Phase 6 정리 시점에 `g_ctrl.force_loadcell` 별도 필드로 옮길 예정.
