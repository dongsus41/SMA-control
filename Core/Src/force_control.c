#include "force_control.h"
#include "controller.h"   /* g_ctrl */

float ForceControl_Calculate(float current_force, uint8_t channel)
{
    Force_PID_TypeDef* p = &g_ctrl.force_params[channel];

    /* safety_mode CRITICAL 이상이면 integral 리셋 후 즉시 반환.
     * Actuator_Apply가 어차피 PWM을 0으로 강제하지만, 여기서도 차단해
     * 안전 모드 해제 직후 integral windup으로 PWM이 튀는 것을 방지. */
    if (g_ctrl.safety_mode[channel] >= SAFETY_MODE_CRITICAL) {
        p->integral   = 0.0f;
        p->last_error = 0.0f;
        return 0.0f;
    }

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
