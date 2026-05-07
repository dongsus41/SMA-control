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

/* Controller_Update는 Task 2에서 추가 */
