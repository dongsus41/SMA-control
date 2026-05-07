#include "temp_control.h"
#include "controller.h"   /* g_ctrl */
#include "main.h"

/* MFSMC 파라미터 — Init_Controller에서 g_ctrl.temp_params[i]에 적용 */
#define MFSMC_LAMBDA_HEAT   3.0f    /* 가열 시 lambda */
#define MFSMC_LAMBDA_COOL   0.0f    /* 냉각 시 lambda */
#define MFSMC_ALPHA         12.0f   /* 시스템 모델 추정치 */
#define MFSMC_GAIN          10.0f   /* 외란 제거 강도 */
#define MFSMC_PHI           30.0f   /* Boundary Layer Thickness */
#define MFSMC_FORCED_COOLING_THRESHOLD  1.0f
#define MAX_PWM_LIMIT       100.0f

/* MFSMC 알고리즘 — g_ctrl.temp_params[channel] 직접 참조 */
float Calculate_Ctrl(float current_temp, uint8_t channel)
{
    PID_Param_TypeDef* p = &g_ctrl.temp_params[channel];

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

    float error = p->setpoint - current_temp;

    /* 강제 냉각 로직 */
    if (error < -MFSMC_FORCED_COOLING_THRESHOLD) {
        p->last_error = error;
        p->u_old = 0.0f;
        return 0.0f;
    }

    float error_dot = (error - p->last_error) / dt;

    float alpha  = MFSMC_ALPHA;
    float K_gain = MFSMC_GAIN;
    float lambda = (error_dot > 0) ? MFSMC_LAMBDA_COOL : MFSMC_LAMBDA_HEAT;

    float u_old = p->u_old;
    float F_hat = error_dot + (alpha * u_old);

    float s = error + (lambda * error_dot);

    /* Boundary layer */
    float sat;
    if      (s >  MFSMC_PHI) sat =  1.0f;
    else if (s < -MFSMC_PHI) sat = -1.0f;
    else                      sat = s / MFSMC_PHI;

    float u = (F_hat + K_gain * sat) / alpha;

    if (u < 0.0f)          u = 0.0f;
    if (u > MAX_PWM_LIMIT) u = MAX_PWM_LIMIT;

    p->last_error = error;
    p->u_old      = u;

    return u;
}
