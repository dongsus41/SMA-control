#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stdbool.h>
#include "system_defs.h"

/* Fan ON/OFF 제어 임계값 (target 대비 차이, °C) */
#define TEMP_HIGH_THRESHOLD 8.0f    /* current >= target+8.0 → fan ON */
#define TEMP_LOW_THRESHOLD  5.0f    /* current <= target+5.0 → fan OFF */

/* PID 파라미터 + 상태 (g_ctrl.temp_params[]에 인스턴스화) */
typedef struct {
    float lambda;            /* kp (MFSMC LAMBDA) */
    float alpha;             /* ki (MFSMC ALPHA) */
    float gain;              /* kd (MFSMC GAIN) */
    float setpoint;
    float u_old;
    float last_error;
    float output_min;
    float output_max;
    float max_temp;
    float critical_temp;
    float sensor_error_temp;
    uint8_t safety_mode;
    uint8_t recovery_needed;
    float last_tracked_temp;
    uint32_t last_tracked_time;
    uint32_t low_rise_time;
    bool rise_rate_monitoring;
} PID_Param_TypeDef;

/** @brief MFSMC PID 계산 — g_ctrl.temp_params[channel] 직접 참조.
 *  @param current_temp 현재 측정 온도
 *  @param channel      0~CTRL_CH-1
 *  @return PWM 출력 (0~MAX_PWM_LIMIT)
 */
float Calculate_Ctrl(float current_temp, uint8_t channel);

#endif /* __PID_H */
