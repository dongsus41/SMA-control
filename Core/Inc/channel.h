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
