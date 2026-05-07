#include "controller.h"
#include "main.h"
#include "system_defs.h"
#include "max31855.h"
#include "temp_control.h"
#include "force_control.h"
#include "loadcell_i2c.h"

/* ── Phase 4 신규 글로벌 ── */
cmd_param_t   g_cmd;
ctrl_state_t  g_ctrl;
state_buf_t   g_state;

/* 기존 글로벌 (Phase 6에서 정리 예정) */
extern System_typedef            system;        /* state_level, pnt_pwm, state_fsw */
extern PID_Manager_typedef       pid;           /* Init_PID_Controllers가 채움 — Init_Controller에서 g_ctrl.temp_params로 복사 */
extern ForceControl_TypeDef      force_ctrl;    /* ForceControl_Calculate가 force_pid 직접 참조 — transitional */
extern MAX31855_typedef          tmc;
extern I2C_HandleTypeDef         hi2c2;

/* ═══════════════ 부팅 시 1회 호출 ═══════════════
 * Phase 4 신규: g_ctrl.temp_params[i]에 PID 초기 gain (MFSMC_LAMBDA_HEAT 등) 복사.
 * 기존 Init_PID_Controllers / ForceControl_Init도 호출 — pid/force_ctrl 글로벌 채워짐
 * (transitional, Phase 6에서 정리 예정).
 */
void Init_Controller(void)
{
	Init_PID_Controllers();   /* pid.params[] 채움 */
	ForceControl_Init();       /* force_ctrl.force_pid 채움 */

	/* 새 모델로 PID 파라미터 복사 */
	for (uint8_t i = 0; i < CTRL_CH; i++) {
		g_ctrl.temp_params[i] = pid.params[i];
	}

	/* 부팅 시 모든 채널 OFF */
	for (uint8_t i = 0; i < CTRL_CH; i++) {
		g_cmd.mode[i] = CH_OFF;
	}
}

/* ═══════════════ fast_tick 본체 — Sensor 단계 ═══════════════
 * Phase 4: g_state.temp[]에 raw 측정값 저장. safety check는 Safety_Update로 분리.
 */
void Sensor_Update(void)
{
	TMC_Scan(CTRL_CH);

	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		g_state.temp[i] = tmc.temp_ext14_raw[i];
	}

	/* Phase 5: 로드셀 12ms-cap 폴링 + ring buffer 누적 */
	LoadCell_Update(&hi2c2, HAL_GetTick());
}

/* ═══════════════ fast_tick 본체 — Safety 단계 ═══════════════
 * 채널별 온도 임계 검사 + 히스테리시스. g_ctrl.safety_mode[i] 갱신.
 * controller는 safety_mode를 모름 — actuator overlay에서 강제 적용.
 *
 * 임계값: warn=80°C, critical=120°C, sensor_err=>200°C or <-50°C
 * 히스테리시스: warn→normal=75°C 미만, critical→normal=30°C 이하
 */
void Safety_Update(void)
{
	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		float t = tmc.temp_ext14[i];
		uint8_t mode = g_ctrl.safety_mode[i];

		if (t > 200.0f || t < -50.0f) {
			mode = 3;  /* sensor error */
		}
		else if (mode == 3) {
			/* 센서 정상 복귀 시 mode 0으로 */
			mode = 0;
		}

		if (mode != 3) {
			if (t >= 120.0f) {
				mode = 2;
			}
			else if (mode == 2 && t <= 30.0f) {
				mode = 0;
			}
			else if (mode != 2) {
				if (t >= 80.0f) {
					mode = 1;
				}
				else if (mode == 1 && t < 75.0f) {
					mode = 0;
				}
			}
		}

		g_ctrl.safety_mode[i] = mode;
	}
}

/* ═══════════════ slow_tick 본체 — Controller 단계 ═══════════════
 * 채널별 g_cmd.mode 디스패치 → g_ctrl.cmd_pwm/cmd_fan 채움.
 * safety 무지 — actuator가 overlay 적용.
 *
 * Force 모드는 단일 로드셀 제약상 동시 1채널만 (UART 디코더에서 검증).
 * startup_phase는 첫 ~1초간 PID 비활성 — 부팅 직후 SMA 가열 방지.
 */
void Controller_Update(void)
{
	static uint8_t startup_counter = 0;
	static uint8_t startup_phase   = 1;
	if (startup_phase) {
		startup_counter++;
		if (startup_counter >= 10) {  /* 약 1초 */
			startup_phase   = 0;
			startup_counter = 0;
		}
	}

	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		switch ((ch_ctrl_e)g_cmd.mode[i])
		{
		case CH_OFF:
			g_ctrl.cmd_pwm[i] = 0;
			g_ctrl.cmd_fan[i] = 0;
			break;

		case CH_MANUAL:
			g_ctrl.cmd_pwm[i] = g_cmd.manual_pwm[i];
			g_ctrl.cmd_fan[i] = g_cmd.manual_fan[i];
			break;

		case CH_TEMP:
			if (startup_phase) {
				g_ctrl.cmd_pwm[i] = 0;
				g_ctrl.cmd_fan[i] = 0;
			} else {
				float current = tmc.temp_ext14[i];
				float target  = (float)g_cmd.target[i] / 4.0f;  /* 0.25°C/LSB */
				g_ctrl.temp_params[i].setpoint = target;

				float ctrl_output = Calculate_Ctrl(&g_ctrl.temp_params[i], current, i);
				if (ctrl_output > 100.0f) ctrl_output = 100.0f;
				if (ctrl_output < 0.0f)   ctrl_output = 0.0f;
				g_ctrl.cmd_pwm[i] = (uint8_t)ctrl_output;

				/* fan 히스테리시스 (기존 Control_Fan_By_Temperature 로직) */
				float diff = current - target;
				if (g_ctrl.cmd_fan[i] == 0 && diff >= TEMP_HIGH_THRESHOLD) {
					g_ctrl.cmd_fan[i] = 1;
				} else if (g_ctrl.cmd_fan[i] == 1 && diff <= TEMP_LOW_THRESHOLD) {
					g_ctrl.cmd_fan[i] = 0;
				}
			}
			break;

		case CH_FORCE:
		{
			/* Phase 5: ring buffer 8샘플 평균 사용. stale 검출 시 안전 차단.
			 * I2C read 자체는 fast_tick(Sensor_Update)에서 12ms-cap으로 이미 진행 중. */
			uint32_t now = HAL_GetTick();
			if (LoadCell_IsStale(now, 100U))  /* 100ms 동안 새 데이터 없으면 stale */
			{
				g_ctrl.cmd_pwm[i] = 0;
				g_ctrl.cmd_fan[i] = 1;
			}
			else
			{
				float force_avg    = LoadCell_GetAverage();
				float target_force = (float)g_cmd.target[i] / 10.0f;  /* 0.1g/LSB */

				/* transitional: ForceControl_Calculate가 force_ctrl.force_pid 직접 참조 */
				force_ctrl.force_pid.target_force = target_force;
				force_ctrl.force_pid.enabled      = 1;

				float ctrl_output = ForceControl_Calculate(force_avg);
				g_ctrl.cmd_pwm[i] = (uint8_t)ctrl_output;
				g_ctrl.cmd_fan[i] = 0;
			}

			/* SendForceState 송신용 캐시 갱신 (transitional, Phase 6 정리 예정) */
			force_ctrl.loadcell.force        = LoadCell_GetAverage();
			force_ctrl.loadcell.displacement = LoadCell_GetLatestDisp();
			force_ctrl.loadcell.valid        = !LoadCell_IsStale(now, 100U);
			break;
		}

		default:
			g_ctrl.cmd_pwm[i] = 0;
			g_ctrl.cmd_fan[i] = 0;
			break;
		}
	}
}

/* ═══════════════ slow_tick 본체 — Actuator 단계 ═══════════════
 * controller가 produce한 cmd_pwm/cmd_fan에 safety overlay 적용.
 * PWM CCR + FSW GPIO 출력 + g_state.pwm/fan에 송신용 값 mirror.
 *
 * Safety overlay (한 곳에서만 강제):
 *   safety_mode 1 (warn)        → fan 강제 ON
 *   safety_mode 2 (critical) /
 *   safety_mode 3 (sensor_err)  → PWM=0, fan 강제 ON
 *
 * fan 송신 코드 매립 (기존 호환):
 *   0  : 정상 OFF
 *   1  : 정상 ON
 *   17 : warn / critical
 *   49 : sensor_err
 */
void Actuator_Apply(void)
{
	for (uint8_t i = 0; i < CTRL_CH; i++)
	{
		uint8_t pwm = g_ctrl.cmd_pwm[i];
		uint8_t fan = g_ctrl.cmd_fan[i];
		uint8_t fan_out_code;

		switch (g_ctrl.safety_mode[i])
		{
		case 0:
			fan_out_code = fan ? 1 : 0;
			break;
		case 1:
			fan = 1;
			fan_out_code = 17;
			break;
		case 2:
			pwm = 0;
			fan = 1;
			fan_out_code = 17;
			break;
		case 3:
		default:
			pwm = 0;
			fan = 1;
			fan_out_code = 49;
			break;
		}

		/* 하드웨어 출력 */
		*system.pnt_pwm[i] = pwm;
		if (fan) FSW_on(i);
		else     FSW_off(i);

		/* 송신용 g_state */
		g_state.pwm[i] = pwm;
		g_state.fan[i] = fan_out_code;

		/* transitional: 일부 잔여 코드가 system.state_fsw / state_pwm 참조할 수 있음 */
		system.state_fsw[i] = fan ? FAN_ON : FAN_OFF;
		system.state_pwm[i] = pwm;
	}
}
