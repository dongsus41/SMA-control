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
 *
 * 부팅 시 1회 호출:
 *   Init_Controller();         // g_ctrl.temp_params/force_params PID gain 초기값
 */

void Init_Controller(void);
void Sensor_Update(void);
void Safety_Update(void);
void Controller_Update(void);
void Actuator_Apply(void);

#endif /* __CONTROLLER_H */
