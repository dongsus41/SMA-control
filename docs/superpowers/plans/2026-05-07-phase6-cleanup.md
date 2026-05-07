# Phase 6: 정리 + channel.h 흡수 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Phase 4에서 도입한 새 모델 (`g_cmd / g_ctrl / g_state`)이 단일 truth가 되도록 잔여 거대 글로벌 (`system / pid / force_ctrl`)과 dead code 함수를 정리. `channel.h`를 `controller.h`에 흡수해 분산 해소.

**Architecture:** 펌웨어 내부 cleanup만 — UART 프로토콜 / PC 코드 변경 없음, 호환성 영향 없음. 각 task가 독립 빌드 가능. 의미 변경 없는 정리 작업 (PID 응답 / 안전 모드 / Force 동작 모두 baseline 동일해야 함).

**Tech Stack:** STM32G474 / STM32CubeIDE.

**참조 문서:** `docs/superpowers/specs/2026-05-07-isr-pipeline-refactor-design.md` 섹션 6 Phase 6

---

## File Structure

| 파일 | 변경 |
|---|---|
| `Core/Inc/channel.h` | **삭제** (controller.h에 흡수) |
| `Core/Inc/controller.h` | channel.h 내용 통합 |
| `Core/Inc/main.h` | dead typedef 제거 (System_typedef, Buf_FDCAN_Tx_*, Ctrl_Param_typedef, Databuf_FDCAN_*, param_*), Manual_Control declaration 제거 |
| `Core/Src/main.c` | system 글로벌 정의 제거, Manual_Control 함수 제거, Flash_* 함수 제거 (사용 안 함), pnt_pwm 매핑 controller로 이동, state_level → g_sys.state_level |
| `Core/Inc/system_defs.h` | g_sys에 `state_level` 필드 추가 |
| `Core/Inc/temp_control.h` | Shared_Data_TypeDef / Buf_FDCAN_PID_Tuning_* / PID_Manager_typedef 제거. Calculate_Ctrl 함수만 유지 |
| `Core/Src/temp_control.c` | dead 함수 8개 제거 (Init_PID_Controllers / Update_Fan_Status / Check_Temperature_Sensor / Check_Safety_Temperature / Check_Temperature_Rise_Rate / Set_PWM_Output / Control_Fan_By_Temperature / PID_Set_Target_Temp). pid 글로벌 제거. Calculate_Ctrl만 유지 |
| `Core/Inc/force_control.h` | ControlMode_TypeDef / ForceControl_TypeDef 제거. ForceControl_Calculate stateless 시그니처 변경 |
| `Core/Src/force_control.c` | force_ctrl 글로벌 제거. ForceControl_Calculate(state*, current) stateless 변환. ForceControl_Init/SetTarget/Enable/Disable 제거 |
| `Core/Src/controller.c` | pnt_pwm 매핑 내장, force_ctrl.* / pid.* 참조 제거. force PID 상태는 g_ctrl.force_params[i] 사용 |
| `Core/Src/uart_protocol.c` | force_ctrl 참조 제거 (UartComm_SendForceState가 LoadCell_Get* 직접 호출) |
| `Core/Src/max31855.c` | system.buf_fdcan_tx 잔재 라인 제거 |

신규 파일 없음. 5개 task 분할.

---

## Task 1: channel.h → controller.h 흡수

가장 안전한 단순 통합. enum + struct + extern 모두 controller.h로 이동, channel.h 삭제.

**Files:**
- Delete: `Core/Inc/channel.h`
- Modify: `Core/Inc/controller.h`
- Modify: `Core/Src/controller.c`, `Core/Src/uart_protocol.c`, `Core/Src/main.c` (channel.h include 제거)

- [ ] **Step 1.1: `controller.h`에 channel.h 내용 통합**

`Core/Inc/controller.h` 전체를 다음으로 교체:

```c
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <stdint.h>
#include "system_defs.h"      /* CTRL_CH */
#include "temp_control.h"     /* PID_Param_TypeDef */
#include "force_control.h"    /* Force_PID_TypeDef */

/*
 * controller.h — 채널 상태 데이터 모델 + 제어 파이프라인 함수.
 *
 * 데이터 모델 (디자인 문서 섹션 3):
 *   g_cmd   : PC 명령 입력 (CMD 0x01 디코드 결과)
 *   g_ctrl  : controller 내부 상태 (PID 파라미터, safety, cmd_pwm/fan)
 *   g_state : MCU → PC 송신 buffer (CMD 0x02)
 *
 * 파이프라인 4단계 (호출 사이트는 main.c의 while(1)):
 *   if (g_sys.fast_tick > 0) {
 *       Sensor_Update();       // SPI 6채널 thermo + 로드셀 12ms-cap polling
 *       Safety_Update();       // g_ctrl.safety_mode[] 갱신
 *       UartComm_SendState();
 *       if (force 활성 채널 있음) UartComm_SendForceState();
 *   }
 *   if (g_sys.slow_tick > 0) {
 *       Controller_Update();   // mode dispatch → g_ctrl.cmd_pwm/cmd_fan
 *       Actuator_Apply();      // safety overlay + PWM CCR + FSW GPIO + g_state.pwm/fan
 *   }
 *
 * 부팅 시 1회 호출:
 *   Init_Controller();         // g_ctrl 초기화 (PID gain 등)
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

/* ── controller 내부 상태 ── */
typedef struct {
    PID_Param_TypeDef   temp_params[CTRL_CH];   /* 온도 PID 파라미터 + 상태 */
    Force_PID_TypeDef   force_params[CTRL_CH];  /* 힘 PID 파라미터 + 상태 (현재 단일 채널만 사용) */
    uint8_t             safety_mode[CTRL_CH];   /* 0=normal / 1=warn(80°C+) / 2=critical(120°C+) / 3=sensor_err */
    uint8_t             cmd_pwm[CTRL_CH];       /* controller → actuator (0~100) */
    uint8_t             cmd_fan[CTRL_CH];       /* controller → actuator (0/1) */
} ctrl_state_t;

/* ── MCU → PC 송신 buffer (CMD 0x02 페이로드, 24B) ──
 * fan에 safety 코드 매립: 0=정상 OFF, 1=정상 ON, 17=warn, 49=sensor_err
 */
typedef struct {
    uint8_t  pwm[CTRL_CH];
    uint8_t  fan[CTRL_CH];
    uint16_t temp[CTRL_CH];
} state_buf_t;

extern cmd_param_t   g_cmd;
extern ctrl_state_t  g_ctrl;
extern state_buf_t   g_state;

/* ── 파이프라인 함수 ── */
void Init_Controller(void);
void Sensor_Update(void);
void Safety_Update(void);
void Controller_Update(void);
void Actuator_Apply(void);

#endif /* __CONTROLLER_H */
```

- [ ] **Step 1.2: `channel.h` 삭제**

```bash
rm Core/Inc/channel.h
```

- [ ] **Step 1.3: 사용처에서 `#include "channel.h"` 라인 제거**

`Core/Src/controller.c`:
```c
#include "controller.h"
#include "channel.h"     /* 제거 */
```

`Core/Src/uart_protocol.c`:
```c
#include "force_control.h"
#include "channel.h"     /* 제거 */
```

`Core/Src/main.c`:
```c
#include "controller.h"
#include "channel.h"     /* 제거 */
```

- [ ] **Step 1.4: 빌드 확인 + commit**

```bash
git rm Core/Inc/channel.h
git add Core/Inc/controller.h Core/Src/controller.c Core/Src/uart_protocol.c Core/Src/main.c
git commit -m "refactor(phase6): channel.h를 controller.h로 흡수

분산 해소 — 데이터 모델 + 파이프라인 함수가 controller.h 한 곳에.
의미 변경 없음."
```

---

## Task 2: pid 글로벌 + temp_control 슬림화

`pid` 글로벌 제거. `Init_PID_Controllers` → `Init_Controller`가 직접 g_ctrl.temp_params 채우도록 변경. dead 함수 8개 제거.

**Files:**
- Modify: `Core/Inc/temp_control.h`
- Modify: `Core/Src/temp_control.c`
- Modify: `Core/Src/controller.c`, `Core/Src/uart_protocol.c`, `Core/Src/force_control.c`, `Core/Src/main.c`

- [ ] **Step 2.1: `temp_control.h` 슬림화**

`Core/Inc/temp_control.h` 전체를 다음으로 교체:

```c
#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stdbool.h>
#include "system_defs.h"

/* Fan 상태값 정의 (CMD 0x02 fan[i] 매립 코드) */
#define FAN_OFF              0   /* 팬 꺼짐 (정상 모드) */
#define FAN_ON               1   /* 팬 켜짐 (정상 모드) */
#define FAN_SAFETY_LEVEL1    17  /* 1단계 안전 모드 (80°C 초과) */
#define FAN_SENSOR_ERROR     49  /* 센서 오류 모드 (200°C 초과 또는 -50°C 미만) */

/* Safety 임계값 */
#define SAFETY_LIMIT_TEMP    80.0f  /* 1단계 안전 모드 진입 (°C) */
#define SAFETY_TARGET_TEMP   70.0f  /* 1단계 안전 모드 복구 (히스테리시스) */

/* Fan ON/OFF 제어 임계값 (target 대비 차이, °C) */
#define TEMP_HIGH_THRESHOLD 8.0f    /* current >= target+8.0 → fan ON */
#define TEMP_LOW_THRESHOLD  5.0f    /* current <= target+5.0 → fan OFF */

/* PID 파라미터 + 상태 (g_ctrl.temp_params[]에 인스턴스화) */
typedef struct {
    float lambda;            /* kp */
    float alpha;             /* ki */
    float gain;              /* kd */
    float setpoint;
    float u_old;
    float last_error;
    float output_min;
    float output_max;
    float max_temp;
    float critical_temp;
    float sensor_error_temp;
    uint8_t safety_mode;       /* legacy 필드 — Phase 4 이후 g_ctrl.safety_mode[] 사용 */
    uint8_t recovery_needed;
    float last_tracked_temp;
    uint32_t last_tracked_time;
    uint32_t low_rise_time;
    bool rise_rate_monitoring;
} PID_Param_TypeDef;

/** @brief MFSMC PID 계산. controller.c가 g_ctrl.temp_params[i] 전달. */
float Calculate_Ctrl(PID_Param_TypeDef* pid, float current_temp, uint8_t channel);

#endif /* __PID_H */
```

(주: PID_Param_TypeDef의 일부 필드는 Calculate_Ctrl에서 사용 안 하지만 기존 호환을 위해 유지. 향후 stateless 더 깔끔하게 정리 가능)

- [ ] **Step 2.2: `temp_control.c` 슬림화**

`Core/Src/temp_control.c` 전체를 다음으로 교체:

```c
#include "temp_control.h"
#include "main.h"

/* MFSMC 파라미터 — Init_Controller에서 g_ctrl.temp_params[i]에 적용 */
#define MFSMC_LAMBDA_HEAT   3.0f    /* 가열 시 lambda */
#define MFSMC_LAMBDA_COOL   0.0f    /* 냉각 시 lambda */
#define MFSMC_ALPHA         12.0f   /* 시스템 모델 추정치 */
#define MFSMC_GAIN          10.0f   /* 외란 제거 강도 */
#define MFSMC_PHI           30.0f   /* Boundary Layer Thickness */
#define MFSMC_FORCED_COOLING_THRESHOLD  1.0f
#define MAX_PWM_LIMIT       100.0f

/* MFSMC 알고리즘 — stateless, 인자만으로 작동 */
float Calculate_Ctrl(PID_Param_TypeDef* pid_param, float current_temp, uint8_t channel)
{
    /* 시간차 (dt) */
    static uint32_t last_call_time[CTRL_CH] = {0};
    uint32_t current_time = HAL_GetTick();
    if (last_call_time[channel] == 0) {
        last_call_time[channel] = current_time;
        return 0.0f;
    }
    float dt = (current_time - last_call_time[channel]) / 1000.0f;
    last_call_time[channel] = current_time;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.1f;

    float error = pid_param->setpoint - current_temp;

    /* 강제 냉각 로직 */
    if (error < -MFSMC_FORCED_COOLING_THRESHOLD) {
        pid_param->last_error = error;
        pid_param->u_old = 0.0f;
        return 0.0f;
    }

    float error_dot = (error - pid_param->last_error) / dt;

    float alpha = MFSMC_ALPHA;
    float K_gain = MFSMC_GAIN;
    float lambda = (error_dot > 0) ? MFSMC_LAMBDA_COOL : MFSMC_LAMBDA_HEAT;

    float u_old = pid_param->u_old;
    float F_hat = error_dot + (alpha * u_old);

    float s = error + (lambda * error_dot);

    /* Boundary layer */
    float sat;
    if (s > MFSMC_PHI)        sat =  1.0f;
    else if (s < -MFSMC_PHI)  sat = -1.0f;
    else                       sat = s / MFSMC_PHI;

    float u = (F_hat + K_gain * sat) / alpha;

    if (u < 0.0f)            u = 0.0f;
    if (u > MAX_PWM_LIMIT)   u = MAX_PWM_LIMIT;

    pid_param->last_error = error;
    pid_param->u_old      = u;

    return u;
}
```

(주: 기존 함수의 본체 그대로. `pid` 글로벌 제거됨. Init_PID_Controllers / Check_* / Update_* / Set_PWM_Output / Control_Fan_By_Temperature / PID_Set_Target_Temp 함수 모두 제거)

- [ ] **Step 2.3: `controller.c` Init_Controller 변경 — Init_PID_Controllers 호출 제거 + 직접 채움**

`Core/Src/controller.c` 의 Init_Controller 함수를 다음으로 교체:

```c
void Init_Controller(void)
{
	/* g_ctrl.temp_params[] PID gain 초기값 (기존 Init_PID_Controllers와 동일 값) */
	for (uint8_t i = 0; i < CTRL_CH; i++) {
		g_ctrl.temp_params[i].lambda            = 3.0f;    /* MFSMC_LAMBDA_HEAT */
		g_ctrl.temp_params[i].alpha             = 12.0f;   /* MFSMC_ALPHA */
		g_ctrl.temp_params[i].gain              = 10.0f;   /* MFSMC_GAIN */
		g_ctrl.temp_params[i].setpoint          = 50.0f;
		g_ctrl.temp_params[i].u_old             = 0.0f;
		g_ctrl.temp_params[i].last_error        = 0.0f;
		g_ctrl.temp_params[i].output_min        = 0.0f;
		g_ctrl.temp_params[i].output_max        = 100.0f;
		g_ctrl.temp_params[i].max_temp          = 80.0f;
		g_ctrl.temp_params[i].critical_temp     = 120.0f;
		g_ctrl.temp_params[i].sensor_error_temp = 200.0f;
	}

	/* 부팅 시 모든 채널 OFF */
	for (uint8_t i = 0; i < CTRL_CH; i++) {
		g_cmd.mode[i] = CH_OFF;
	}

	/* force_params 초기화는 Task 3에서 추가 */
}
```

- [ ] **Step 2.4: `controller.c`의 pid extern 제거**

`Core/Src/controller.c` 상단의 extern 선언:

```c
extern PID_Manager_typedef       pid;           /* 제거 */
```

삭제.

- [ ] **Step 2.5: `force_control.c`의 `pid.enable_pid` 참조 제거**

`Core/Src/force_control.c`의 `ForceControl_Enable` 함수 안:

```c
    // 해당 채널의 온도 PID 비활성화 (모드 충돌 방지)
    pid.enable_pid[channel] = 0;
```

이 라인 제거 (이 함수 자체는 dead code지만 Task 3에서 함수 전체 제거 예정).

- [ ] **Step 2.6: 빌드 확인 + commit**

```bash
git add Core/Inc/temp_control.h Core/Src/temp_control.c Core/Src/controller.c Core/Src/force_control.c
git commit -m "refactor(phase6): pid 글로벌 + temp_control.c 슬림화

- pid (PID_Manager_typedef) 글로벌 제거
- Init_PID_Controllers, Check_*, Update_*, Set_PWM_Output,
   Control_Fan_By_Temperature, PID_Set_Target_Temp 함수 제거
- Init_Controller가 g_ctrl.temp_params 직접 채움
- Calculate_Ctrl는 stateless로 유지 (channel.c에서 인자 전달)

dead code 정리. 의미 변경 없음."
```

---

## Task 3: force_ctrl 글로벌 + force_control 정리

`force_ctrl` 글로벌 제거. `ForceControl_Calculate(state*, current_force)` stateless 변환. dead 함수 4개 제거.

**Files:**
- Modify: `Core/Inc/force_control.h`
- Modify: `Core/Src/force_control.c`
- Modify: `Core/Src/controller.c`
- Modify: `Core/Src/uart_protocol.c`

- [ ] **Step 3.1: `force_control.h` 슬림화**

`Core/Inc/force_control.h` 전체를 다음으로 교체:

```c
#ifndef __FORCE_CONTROL_H
#define __FORCE_CONTROL_H

#include "main.h"

/* PID 파라미터 + 상태 (g_ctrl.force_params[]에 인스턴스화) */
typedef struct {
    float kp;
    float ki;
    float kd;
    float target_force;         /* 목표 힘 (g 또는 N) */
    float integral;
    float last_error;
    float output_min;
    float output_max;
    float max_force;            /* 안전 상한 */
    uint8_t enabled;
} Force_PID_TypeDef;

/** @brief Force PID 계산 — stateless, 인자만으로 작동.
 *  controller.c가 g_ctrl.force_params[i] 전달.
 *  @param p             PID 파라미터 + 상태 (integral / last_error 갱신됨)
 *  @param current_force 현재 측정 힘 (g, ring buffer 평균값)
 *  @return PWM 출력 (0~output_max)
 */
float ForceControl_Calculate(Force_PID_TypeDef* p, float current_force);

#endif
```

(ControlMode_TypeDef enum, ForceControl_TypeDef 구조체, ForceControl_Init / SetTarget / Enable / Disable 모두 제거)

- [ ] **Step 3.2: `force_control.c` 슬림화**

`Core/Src/force_control.c` 전체를 다음으로 교체:

```c
#include "force_control.h"

float ForceControl_Calculate(Force_PID_TypeDef* p, float current_force)
{
    float error = p->target_force - current_force;

    /* 적분항 (anti-windup) */
    p->integral += error;
    float integral_limit = (p->ki != 0.0f) ? (p->output_max / p->ki) : 0.0f;
    if (p->integral > integral_limit)  p->integral =  integral_limit;
    if (p->integral < -integral_limit) p->integral = -integral_limit;

    /* 미분항 */
    float derivative = error - p->last_error;
    p->last_error = error;

    /* PID 출력 */
    float output = (p->kp * error) + (p->ki * p->integral) + (p->kd * derivative);

    if (output > p->output_max) output = p->output_max;
    if (output < p->output_min) output = p->output_min;

    return output;
}
```

(force_ctrl 글로벌 제거. Init / SetTarget / Enable / Disable 함수 제거)

- [ ] **Step 3.3: `controller.c`의 Init_Controller에 force_params 초기화 추가**

`Core/Src/controller.c`의 Init_Controller에 다음 추가 (temp_params 초기화 다음):

```c
	/* g_ctrl.force_params[] 초기값 (기존 ForceControl_Init와 동일) */
	for (uint8_t i = 0; i < CTRL_CH; i++) {
		g_ctrl.force_params[i].kp           = 2.0f;
		g_ctrl.force_params[i].ki           = 0.5f;
		g_ctrl.force_params[i].kd           = 0.1f;
		g_ctrl.force_params[i].target_force = 0.0f;
		g_ctrl.force_params[i].integral     = 0.0f;
		g_ctrl.force_params[i].last_error   = 0.0f;
		g_ctrl.force_params[i].output_min   = 0.0f;
		g_ctrl.force_params[i].output_max   = 100.0f;
		g_ctrl.force_params[i].max_force    = 500.0f;
		g_ctrl.force_params[i].enabled      = 0;
	}
```

- [ ] **Step 3.4: `controller.c`의 CH_FORCE 분기에서 force_ctrl 참조 제거**

`Core/Src/controller.c`의 CH_FORCE case 블록을 다음으로 교체:

```c
		case CH_FORCE:
		{
			/* Phase 5 ring buffer 평균 + Phase 6 stateless ForceControl_Calculate. */
			uint32_t now = HAL_GetTick();
			if (LoadCell_IsStale(now, 100U))
			{
				g_ctrl.cmd_pwm[i] = 0;
				g_ctrl.cmd_fan[i] = 1;
			}
			else
			{
				float force_avg = LoadCell_GetAverage();
				g_ctrl.force_params[i].target_force = (float)g_cmd.target[i] / 10.0f;  /* 0.1g/LSB */
				g_ctrl.force_params[i].enabled      = 1;

				float ctrl_output = ForceControl_Calculate(&g_ctrl.force_params[i], force_avg);
				g_ctrl.cmd_pwm[i] = (uint8_t)ctrl_output;
				g_ctrl.cmd_fan[i] = 0;
			}
			break;
		}
```

(force_ctrl.loadcell 캐시 갱신 라인 제거 — UartComm_SendForceState가 LoadCell_Get* 직접 호출)

- [ ] **Step 3.5: `controller.c`의 force_ctrl extern 제거**

`Core/Src/controller.c` 상단:

```c
extern ForceControl_TypeDef      force_ctrl;    /* 제거 */
```

- [ ] **Step 3.6: `uart_protocol.c`의 SendForceState 변경 — LoadCell_Get* 직접 호출**

`Core/Src/uart_protocol.c`의 `UartComm_SendForceState`를 다음으로 교체:

```c
void UartComm_SendForceState(void)
{
    int8_t force_ch = -1;
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        if (g_cmd.mode[i] == CH_FORCE) {
            force_ch = (int8_t)i;
            break;
        }
    }
    if (force_ch < 0) return;

    float force = LoadCell_GetAverage();
    float disp  = LoadCell_GetLatestDisp();

    uint8_t payload[FORCE_STATE_PAYLOAD_SIZE];
    payload[0] = CH_FORCE;
    payload[1] = (uint8_t)force_ch;
    memcpy(&payload[2], &force, 4);
    memcpy(&payload[6], &disp,  4);
    UartComm_SendFrame(CMD_FORCE_STATE, payload, FORCE_STATE_PAYLOAD_SIZE);
}
```

`Core/Src/uart_protocol.c` 상단의 extern:

```c
extern ForceControl_TypeDef force_ctrl; /* 제거 */
```

include에 loadcell_i2c.h 있는지 확인 (`#include "loadcell_i2c.h"` 이미 있음).

- [ ] **Step 3.7: 빌드 확인 + commit**

```bash
git add Core/Inc/force_control.h Core/Src/force_control.c Core/Src/controller.c Core/Src/uart_protocol.c
git commit -m "refactor(phase6): force_ctrl 글로벌 + force_control.c 정리

- force_ctrl (ForceControl_TypeDef) 글로벌 제거
- ForceControl_Calculate stateless 변환 (state* 인자 받음)
- ForceControl_Init / SetTarget / Enable / Disable 제거
- ControlMode_TypeDef enum 제거
- Init_Controller가 g_ctrl.force_params[] 직접 채움
- UartComm_SendForceState가 LoadCell_Get* 직접 호출 (force_ctrl.loadcell 캐시 제거)

dead code 정리. 의미 변경 없음."
```

---

## Task 4: system 글로벌 정리 + main.c / main.h 슬림화

가장 침투적. system_typedef 제거, state_level은 g_sys로 이동, pnt_pwm 매핑은 controller.c로 이동, Manual_Control / Flash_* / dead typedef 제거.

**Files:**
- Modify: `Core/Inc/system_defs.h` (g_sys에 state_level 추가)
- Modify: `Core/Inc/main.h` (dead typedef 제거)
- Modify: `Core/Src/main.c` (system 글로벌 제거, Manual_Control 제거, Flash_* 제거, pnt_pwm 매핑 이동)
- Modify: `Core/Src/controller.c` (pnt_pwm 매핑 내장)
- Modify: `Core/Src/max31855.c` (system.buf_fdcan_tx 잔재 라인 제거)
- Modify: `Core/Src/uart_protocol.c` (system.state_level → g_sys.state_level)

- [ ] **Step 4.1: `system_defs.h`에 state_level 필드 추가**

`Core/Inc/system_defs.h`의 SystemState_t 안에 `state_command` 다음에 추가:

```c
    uint8_t           state_command;        /**< (legacy) */

    uint8_t           state_level;          /**< Phase 6: 시스템 상태 (0=INIT, 1=READY, 2=GO) */
```

`SYSTEM_INIT/READY/GO` enum은 main.h에 있으므로 system_defs.h에서 사용하려면 forward 선언 또는 single source. 가장 깔끔한 건 enum을 system_defs.h로 이동.

`Core/Inc/system_defs.h`의 SystemState_t **위**에 추가:

```c
typedef enum {
    SYSTEM_INIT  = 0,
    SYSTEM_READY = 1,
    SYSTEM_GO    = 2
} System_State_typedef;
```

그리고 SystemState_t의 state_level 타입을 변경:

```c
    System_State_typedef state_level;
```

- [ ] **Step 4.2: `main.h` 슬림화**

`Core/Inc/main.h` 의 `/* USER CODE BEGIN Private defines */` ~ `/* USER CODE END Private defines */` 영역을 다음으로 교체:

```c
/* USER CODE BEGIN Private defines */
/* System_State_typedef은 system_defs.h로 이동 */

/* Phase 6: System_typedef / Buf_FDCAN_Tx_* / Ctrl_Param_typedef /
 * Databuf_FDCAN_typedef / param_struct / param_union — 모두 제거
 * (g_cmd / g_ctrl / g_state로 대체됨)
 */
/* USER CODE END Private defines */
```

또한 `/* USER CODE BEGIN ET */` 안의 `Databuf_FDCAN_typedef` 제거:

```c
/* USER CODE BEGIN ET */
/* Phase 6: Databuf_FDCAN_typedef 제거 (FDCAN 미사용) */
/* USER CODE END ET */
```

`/* USER CODE BEGIN EFP */` 안의 `Manual_Control` declaration 제거:

```c
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */
```

`/* USER CODE BEGIN Includes */` 안에 controller.h 추가하거나 그대로 두기 (main.c가 직접 include하므로 불필요):

기존:
```c
#include "common_defs.h"
#include "fdcan.h"
#include "max31855.h"
#include "temp_control.h"
```

다음으로 변경:
```c
/* common_defs.h은 system_defs.h로 리네임 후 main.c에서 직접 include */
#include "fdcan.h"
#include "max31855.h"
#include "temp_control.h"
```

(`common_defs.h`는 이미 삭제됐고 `system_defs.h`로 대체됐으므로 이 라인 자체 제거)

- [ ] **Step 4.3: `main.c`에서 system 글로벌 + Manual_Control + Flash_* 제거**

`Core/Src/main.c` 상단의 `/* USER CODE BEGIN PV */` 영역을 다음으로 교체:

```c
/* USER CODE BEGIN PV */
/* Phase 6: system / pid / force_ctrl 글로벌 모두 controller.c 또는 모듈 내부로 이동.
 * 잔여: tmc는 max31855.c, hi2c2는 main.c가 정의 (HAL).
 */
extern MAX31855_typedef tmc;
/* USER CODE END PV */
```

`Manual_Control` 함수 정의(현재 ~150줄) **전체 삭제**.

`Flash_Erase / Flash_Write / Flash_Read` 함수도 사용처 0이면 삭제 (확인 후). 사용처 있으면 그대로.

```bash
grep -rn "Flash_Erase\|Flash_Write\|Flash_Read\|EraseInitStruct\|param_flash\|param\." Core/Src/ | grep -v Flash.c
```

이 함수들 사용 안 하면:
- `Flash_Erase`, `Flash_Write`, `Flash_Read` 함수 정의 제거
- `static FLASH_EraseInitTypeDef EraseInitStruct;`, `param_union param;`, `param_union param_flash;` 변수 제거

- [ ] **Step 4.4: `main.c`의 pnt_pwm 매핑을 Init_Controller로 이동**

`Core/Src/main.c`의 main() 함수 안의 다음 코드:

```c
	system.pnt_pwm[0] = &TIM2->CCR4;
	system.pnt_pwm[1] = &TIM2->CCR1;
	system.pnt_pwm[2] = &TIM3->CCR1;
	system.pnt_pwm[3] = &TIM3->CCR2;
	system.pnt_pwm[4] = &TIM3->CCR3;
	system.pnt_pwm[5] = &TIM3->CCR4;
```

**삭제** (Init_Controller가 controller.c 내부 static 매핑으로 처리).

`Core/Src/controller.c`에 추가 (글로벌 정의 다음, Init_Controller 위):

```c
/* PWM CCR 매핑 (channel → TIM CCR pointer). Init_Controller에서 채워짐. */
static volatile uint32_t* s_pwm_ccr[CTRL_CH] = { NULL };

static void controller_init_pwm_ccr(void)
{
	s_pwm_ccr[0] = &TIM2->CCR4;
	s_pwm_ccr[1] = &TIM2->CCR1;
	s_pwm_ccr[2] = &TIM3->CCR1;
	s_pwm_ccr[3] = &TIM3->CCR2;
	s_pwm_ccr[4] = &TIM3->CCR3;
	s_pwm_ccr[5] = &TIM3->CCR4;
}
```

`Init_Controller` 함수 시작 부분에 호출 추가:

```c
void Init_Controller(void)
{
	controller_init_pwm_ccr();

	/* g_ctrl.temp_params[] PID gain 초기값 */
	... (기존 코드 그대로)
```

`Actuator_Apply` 안의 `*system.pnt_pwm[i] = pwm;`를 다음으로 교체:

```c
		*s_pwm_ccr[i] = pwm;
```

- [ ] **Step 4.5: `main.c`에서 system.* 사용처 정리 (state_level만 g_sys로)**

`Core/Src/main.c`의 system.state_level 참조 모두 g_sys.state_level로 교체:

```c
	if (system.state_level == SYSTEM_READY)        /* → g_sys.state_level */
	system.state_level = SYSTEM_GO;                /* → g_sys.state_level */
	system.state_level = SYSTEM_READY;             /* → g_sys.state_level */
	system.state_level = SYSTEM_GO;                /* → g_sys.state_level */
```

`system.n_rx_motion_limit = 1;` 라인 삭제.

기타 main.c 안의 system.* 참조 (state_pwm / state_fsw / buf_fdcan_tx / ctrl_param_now / ctrl_param_save 등)는 모두 Manual_Control 함수 안에 있었으므로 그 함수 삭제 시 사라짐.

- [ ] **Step 4.6: `controller.c`의 transitional 라인 제거 + system extern 제거**

`Core/Src/controller.c`의 Actuator_Apply 끝부분:

```c
		/* transitional: 일부 잔여 코드가 system.state_fsw / state_pwm 참조할 수 있음 */
		system.state_fsw[i] = fan ? FAN_ON : FAN_OFF;
		system.state_pwm[i] = pwm;
```

이 3줄 **삭제**.

`Core/Src/controller.c` 상단의 extern:

```c
extern System_typedef            system;        /* 제거 */
```

(controller.c가 더 이상 system 사용 안 함)

- [ ] **Step 4.7: `uart_protocol.c`의 system.state_level → g_sys.state_level**

`Core/Src/uart_protocol.c`의 `UartComm_HandleControl` 안:

```c
    if (system.state_level != SYSTEM_GO)    /* → g_sys.state_level */
```

상단 extern:

```c
extern System_typedef       system;     /* 제거 */
```

- [ ] **Step 4.8: `max31855.c`의 system.buf_fdcan_tx 잔재 제거**

`Core/Src/max31855.c` 90~91줄:

```c
	system.buf_fdcan_tx.struc.temp[ch] = tmc.temp_ext14_raw[ch];
//	system.buf_fdcan_tx.struc.temp[ch] = tmc.temp_int12_raw[ch];
```

**삭제** (g_state.temp[]는 Sensor_Update가 채움).

- [ ] **Step 4.9: 빌드 확인 + commit**

빌드 시 잔여 system 참조 발견되면 grep으로 사용처 확인 후 정리:

```bash
grep -rn "system\." Core/ | grep -v "system_defs\|stm32g4xx\|HAL_"
```

```bash
git add -A Core/
git commit -m "refactor(phase6): system 글로벌 제거 + main.c/h 슬림화

- system (System_typedef) 글로벌 제거
- state_level → g_sys.state_level (system_defs.h)
- System_State_typedef enum → system_defs.h로 이동
- pnt_pwm[] 매핑 → controller.c 내부 static
- main.c: Manual_Control 함수 제거, Flash_* 함수 제거 (사용 안 함)
- main.h: System_typedef / Buf_FDCAN_Tx_* / Ctrl_Param_typedef /
   Databuf_FDCAN_typedef / param_struct / param_union / Manual_Control
   declaration 모두 제거
- max31855.c: system.buf_fdcan_tx 잔재 라인 제거
- Actuator_Apply: system.state_fsw / state_pwm 갱신 transitional 제거

dead code 정리. 의미 변경 없음."
```

---

## Task 5: 검증 및 finalization

- [ ] **Step 5.1: 전체 grep으로 잔여 dead 참조 확인**

```bash
cd "C:\Users\Dongsu\Desktop\SMA-control"
grep -rn "system\.\|pid\.\|force_ctrl\.\|Manual_Control\|Set_PWM_Output\|Update_Fan_Status\|Control_Fan_By_Temperature\|Check_Temperature_Sensor\|Check_Safety_Temperature\|Check_Temperature_Rise_Rate\|ForceControl_Init\|ForceControl_SetTarget\|ForceControl_Enable\|ForceControl_Disable\|Init_PID_Controllers\|PID_Manager_typedef\|System_typedef\|Buf_FDCAN_Tx_typedef\|Ctrl_Param_typedef\|Databuf_FDCAN_typedef\|param_struct\|param_union" Core/Inc/ Core/Src/
```

매칭 0개여야 정상. 매칭 발견되면 case-by-case 정리.

(`system_defs.h` 자체의 SystemState_t 정의는 매칭되지만 OK — 그건 g_sys의 typedef)

- [ ] **Step 5.2: 빌드 확인 (사용자 수행)**

다른 로컬에서 클론/풀 후 STM32CubeIDE 빌드. 0 errors, unused warning 대폭 감소 (dead code 제거됨).

- [ ] **Step 5.3: 플래시 + 검증 (사용자 수행)**

**Phase 4 / 5 누적 검증과 함께 수행** — Phase 6는 의미 변경 없는 cleanup이므로 동작 baseline 동일해야 함.

| # | 시나리오 | 확인 포인트 |
|---|---|---|
| 1 | 부팅 | LED1 on, "SYSTEM_GO" 진입 |
| 2 | Manual 모드 | PWM/FAN 명령 즉시 반영 |
| 3 | Temp PID | 응답 baseline 동일 |
| 4 | Force 모드 | ring buffer 평균, 응답 동일 |
| 5 | 안전 모드 | 80°C 초과 fan 강제 ON |
| 6 | startup_phase | 부팅 1초간 PID 비활성 |
| 7 | overrun / dropped 카운터 | 0 유지 |

---

## 완료 후 상태

Phase 6 완료 시:
- 거대 글로벌 (`system / pid / force_ctrl`) 모두 제거. 단일 truth: `g_cmd / g_ctrl / g_state` (+ `g_sys`, `tmc`, `s_pwm_ccr`)
- Dead code 함수 ~10개 제거, dead typedef ~6개 제거
- `main.c` 대폭 슬림화 (~250줄 감소 예상)
- `temp_control.c` ~250줄 → ~80줄, `force_control.c` ~90줄 → ~30줄
- `channel.h` 흡수로 분산 해소 — 데이터 모델은 controller.h 한 곳에

리팩토링 시리즈 완료. 단방향 파이프라인 + 채널별 모드 모델이 전체 코드베이스의 단일 패턴으로 자리잡음.

---

## 자체 검토 결과

**Spec coverage**: 디자인 spec 섹션 6 Phase 6 (거대 글로벌 마무리) — Task 2~4가 정확히 커버. 추가로 사용자가 언급한 channel.h 흡수는 Task 1.

**Placeholder scan**: TBD/TODO 없음. 모든 코드 블록 완전. 일부 step에서 grep 결과 기반 case-by-case 정리 명시 — placeholder 아니라 verification 단계.

**Type consistency**:
- `g_cmd / g_ctrl / g_state` extern 통일 (controller.h)
- `Init_Controller` 단일 entry point — temp_params + force_params + pwm_ccr 모두 초기화
- `Calculate_Ctrl(PID_Param_TypeDef*, ...)`, `ForceControl_Calculate(Force_PID_TypeDef*, ...)` 모두 stateless 시그니처

**범위 / 안전성**:
- 4개 task로 분할 — 각 task 독립 빌드 + commit. 회귀 시 의심 범위 좁음.
- Task 1 (channel.h 흡수)이 가장 안전 — 단순 통합. 먼저 검증 후 다음.
- Task 4 (system 정리)가 가장 침투적 — 잔여 grep 단계 (Step 5.1)로 누락 검증.
- 의미 변경 없음 강조 — dead code 제거만.
