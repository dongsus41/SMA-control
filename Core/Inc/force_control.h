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

/** @brief Force PID 계산 — g_ctrl.force_params[channel] 직접 참조.
 *  @param current_force ring buffer 평균 force (g)
 *  @param channel       0~CTRL_CH-1
 *  @return PWM 출력 (0~output_max)
 */
float ForceControl_Calculate(float current_force, uint8_t channel);

#endif
