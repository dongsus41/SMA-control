#include "controller.h"
#include "main.h"
#include "system_defs.h"
#include "max31855.h"
#include "temp_control.h"
#include "force_control.h"
#include "loadcell_i2c.h"

/* 기존 글로벌 참조 (Phase 4에서 정식 모델로 교체 예정) */
extern System_typedef            system;
extern PID_Manager_typedef       pid;
extern ForceControl_TypeDef      force_ctrl;
extern MAX31855_typedef          tmc;
extern I2C_HandleTypeDef         hi2c2;

/* ═══════════════ fast_tick (~250Hz) 본체 ═══════════════
 * Phase 3: do_fast_tick 본체 그대로 이동. 송신 호출은 호출 사이트로 분리.
 */
void Sensor_Update(void)
{
	TMC_Scan(CTRL_CH);

	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		pid.shared_data.temp_data[i] = tmc.temp_ext14[i];

		// 센서 유효성 즉시 검사
		Check_Temperature_Sensor(i, pid.shared_data.temp_data[i]);

		system.buf_fdcan_tx.struc.fan[i]  = system.state_fsw[i];
		system.buf_fdcan_tx.struc.pwm[i]  = *system.pnt_pwm[i];
		system.buf_fdcan_tx.struc.temp[i] = tmc.temp_ext14_raw[i];
	}

	// 타임스탬프 및 플래그 설정
	pid.shared_data.temp_timestamp = HAL_GetTick();
	pid.shared_data.new_temp_data  = 1;
}

/* ═══════════════ slow_tick (~10Hz) 본체 ═══════════════
 * Phase 3: do_slow_tick 본체 그대로 이동.
 * startup_counter는 함수 내 static으로 보존 — 기존과 동일 동작.
 */
void Controller_Update(void)
{
	static uint8_t startup_counter = 0;
	if (pid.startup_phase) {
	    startup_counter++;
	    if (startup_counter >= 10) {  // 10번의 slow_tick 후 (약 1초)
	        pid.startup_phase = 0;
	        startup_counter = 0;
	    }
	}

	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		// ★ 힘 제어
		if (force_ctrl.mode == CTRL_MODE_FORCE && force_ctrl.force_pid.enabled && i == force_ctrl.active_channel)
		{
			// I2C로 로드셀 데이터 읽기
			LoadCell_Read(&hi2c2, &force_ctrl.loadcell);

			if (force_ctrl.loadcell.valid)
			{
				float ctrl_output = ForceControl_Calculate(force_ctrl.loadcell.force);
				Set_PWM_Output(i, (uint8_t)ctrl_output);
			}
			else
			{
				// 센서 에러 시 안전 장치
				Set_PWM_Output(i, 0);
			}
			continue; // 이 채널은 온도제어 건너뜀
		}

		// ★ 온도제어
		// 1. 안전 온도 확인 (최우선 처리)
		bool in_safety_mode = Check_Safety_Temperature(i, pid.shared_data.temp_data[i]);

		// 2. PID 활성화 상태 및 안전 모드 확인
		if (pid.enable_pid[i] && !in_safety_mode)
		{
			float current_temp = pid.shared_data.temp_data[i];
			float target_temp  = pid.params[i].setpoint;

			// 센서 이상 확인
			// bool sensor_ok = Check_Temperature_Rise_Rate(i, current_temp);
			bool sensor_ok = true; // 온도 상승 안전모드 일단 주석처리

			if (sensor_ok)
			{
				// 제어 연산 수행
				float ctrl_output = Calculate_Ctrl(&pid.params[i], current_temp, i);
				Set_PWM_Output(i, (uint8_t)ctrl_output);

				// 온도 기반 팬 제어
				Control_Fan_By_Temperature(i, current_temp, target_temp);
			}
		}
	}
}
