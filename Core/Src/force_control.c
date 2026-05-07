#include "force_control.h"
#include "controller.h"   /* g_ctrl */

float ForceControl_Calculate(float current_force, uint8_t channel)
{
    Force_PID_TypeDef* p = &g_ctrl.force_params[channel];

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
