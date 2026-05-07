#ifndef __CONTROLLER_H
#define __CONTROLLER_H

/*
 * controller.c — 제어 파이프라인 단계별 함수 모음.
 *
 * Phase 3: do_fast_tick / do_slow_tick 본체를 main.c에서 분리.
 *   기존 글로벌(system / pid / force_ctrl / tmc) 그대로 사용.
 *   2함수 (Sensor_Update / Controller_Update)만 분리.
 *
 * Phase 4 예정: g_cmd / g_ctrl / g_state 데이터 모델 도입과 함께
 *   Safety_Update / Actuator_Apply 정식 분리.
 *
 * 호출 사이트: main.c의 while(1) 루프
 *   if (g_sys.fast_tick > 0) → Sensor_Update + UartComm_SendState + (force) UartComm_SendForceState
 *   if (g_sys.slow_tick > 0) → Controller_Update
 */

/** @brief fast_tick (~250Hz) 본체.
 *  - MAX31855 SPI 6채널 스캔
 *  - 채널별 센서 유효성 검사 (Check_Temperature_Sensor)
 *  - PC 송신 buffer (system.buf_fdcan_tx) 채우기
 *  - 타임스탬프 / new_temp_data flag 설정
 *  송신(UartComm_SendState 등)은 호출 사이트(main.c)에서 직접 수행.
 */
void Sensor_Update(void);

/** @brief slow_tick (~10Hz) 본체.
 *  - startup_phase 카운터 진행
 *  - 채널별 force / temp PID 계산 + Set_PWM_Output + 팬 제어
 */
void Controller_Update(void);

#endif /* __CONTROLLER_H */
