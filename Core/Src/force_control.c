#include "force_control.h"
#include <stdio.h>

ForceControl_TypeDef force_ctrl;

extern System_typedef system;

// 기본 힘 제어 PID 게인 (튜닝 필요)
#define FORCE_KP_DEFAULT    2.0f
#define FORCE_KI_DEFAULT    0.5f
#define FORCE_KD_DEFAULT    0.1f
#define FORCE_MAX_DEFAULT   500.0f   // 최대 허용 힘 (g)

void ForceControl_Init(void)
{
    force_ctrl.mode = CTRL_MODE_TEMPERATURE;  // 기본은 온도 모드
    force_ctrl.active_channel = 0;

    force_ctrl.force_pid.kp = FORCE_KP_DEFAULT;
    force_ctrl.force_pid.ki = FORCE_KI_DEFAULT;
    force_ctrl.force_pid.kd = FORCE_KD_DEFAULT;
    force_ctrl.force_pid.target_force = 0.0f;
    force_ctrl.force_pid.integral = 0.0f;
    force_ctrl.force_pid.last_error = 0.0f;
    force_ctrl.force_pid.output_min = 0.0f;
    force_ctrl.force_pid.output_max = 100.0f;
    force_ctrl.force_pid.max_force = FORCE_MAX_DEFAULT;
    force_ctrl.force_pid.enabled = 0;

    LoadCell_Init(&force_ctrl.loadcell);
}

float ForceControl_Calculate(float current_force)
{
    Force_PID_TypeDef *p = &force_ctrl.force_pid;

    float error = p->target_force - current_force;

    // 적분항 (anti-windup)
    p->integral += error;
    float integral_limit = p->output_max / p->ki;
    if (p->integral > integral_limit) p->integral = integral_limit;
    if (p->integral < -integral_limit) p->integral = -integral_limit;

    // 미분항
    float derivative = error - p->last_error;
    p->last_error = error;

    // PID 출력
    float output = (p->kp * error) + (p->ki * p->integral) + (p->kd * derivative);

    // 출력 제한
    if (output > p->output_max) output = p->output_max;
    if (output < p->output_min) output = p->output_min;

    return output;
}

void ForceControl_SetTarget(float target_force)
{
    if (target_force > force_ctrl.force_pid.max_force)
        target_force = force_ctrl.force_pid.max_force;
    force_ctrl.force_pid.target_force = target_force;
    printf("Force target set: %.2f\r\n", target_force);
}

void ForceControl_Enable(uint8_t channel)
{
    force_ctrl.mode = CTRL_MODE_FORCE;
    force_ctrl.active_channel = channel;
    force_ctrl.force_pid.enabled = 1;
    force_ctrl.force_pid.integral = 0.0f;
    force_ctrl.force_pid.last_error = 0.0f;

    /* Phase 6: 이 함수 자체가 dead code — Task 3에서 함수 전체 제거 예정.
     * pid.enable_pid 참조는 Phase 6 Task 2에서 제거. */
    printf("Force control enabled on CH %u\r\n", channel);
}

void ForceControl_Disable(void)
{
    uint8_t ch = force_ctrl.active_channel;
    force_ctrl.mode = CTRL_MODE_TEMPERATURE;
    force_ctrl.force_pid.enabled = 0;

    // PWM 0으로 안전 정지
    *system.pnt_pwm[ch] = 0;
    system.state_pwm[ch] = 0;

    printf("Force control disabled\r\n");
}