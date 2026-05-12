#ifndef __SYSTEM_DEFS_H
#define __SYSTEM_DEFS_H

#define SERIAL_HADD					(&huart3)	//JETSON

#define CAN3_RXID_COMMAND 			0x400
#define CAN3_RXID_GAIN_UPDATE 		0x402
#define CAN3_RXADD_N				2

#define CAN3_TXID_STATE				0x401

#define CTRL_CH						6

#define SWAP32(d)	((((d) & 0xff000000) >> 24)|(((d) & 0x00ff0000) >> 8)|(((d) & 0x0000ff00) << 8)|(((d) & 0x000000ff) << 24))

//#define GPIO_WRITE_0(PORTx, PINx)	(PORTx->BSRR = (uint32_t)PINx << 16U)
#define GPIO_WRITE_0(PORTx, PINx)	(PORTx->BRR = PINx)
#define GPIO_WRITE_1(PORTx, PINx)	(PORTx->BSRR = PINx)
#define GPIO_TOGGLE(PORTx, PINx)	(PORTx->BSRR = (((PORTx->ODR) & PINx) << 16U) | (~(PORTx->ODR) & PINx))
#define GPIO_READ(GPIOx, GPIO_Pin)		(((GPIOx->IDR & GPIO_Pin) != 0x00U) ? 1U : 0U)

#define LED1_on				GPIO_WRITE_0(LED1_GPIO_Port, LED1_Pin)
#define LED1_off			GPIO_WRITE_1(LED1_GPIO_Port, LED1_Pin)
#define LED1_toggle			GPIO_TOGGLE(LED1_GPIO_Port, LED1_Pin)
#define LED2_on				GPIO_WRITE_0(LED2_GPIO_Port, LED2_Pin)
#define LED2_off			GPIO_WRITE_1(LED2_GPIO_Port, LED2_Pin)
#define LED2_toggle			GPIO_TOGGLE(LED2_GPIO_Port, LED2_Pin)

#define FSW0_on				GPIO_WRITE_1(FSW0_GPIO_Port, FSW0_Pin)
#define FSW0_off			GPIO_WRITE_0(FSW0_GPIO_Port, FSW0_Pin)
#define FSW1_on				GPIO_WRITE_1(FSW1_GPIO_Port, FSW1_Pin)
#define FSW1_off			GPIO_WRITE_0(FSW1_GPIO_Port, FSW1_Pin)
#define FSW2_on				GPIO_WRITE_1(FSW2_GPIO_Port, FSW2_Pin)
#define FSW2_off			GPIO_WRITE_0(FSW2_GPIO_Port, FSW2_Pin)
#define FSW3_on				GPIO_WRITE_1(FSW3_GPIO_Port, FSW3_Pin)
#define FSW3_off			GPIO_WRITE_0(FSW3_GPIO_Port, FSW3_Pin)
#define FSW4_on				GPIO_WRITE_1(FSW4_GPIO_Port, FSW4_Pin)
#define FSW4_off			GPIO_WRITE_0(FSW4_GPIO_Port, FSW4_Pin)
#define FSW5_on				GPIO_WRITE_1(FSW5_GPIO_Port, FSW5_Pin)
#define FSW5_off			GPIO_WRITE_0(FSW5_GPIO_Port, FSW5_Pin)

#define PWM0_TIM			htim2
#define PWM1_TIM			htim2
#define PWM2_TIM			htim3
#define PWM3_TIM			htim3
#define PWM4_TIM			htim3
#define PWM5_TIM			htim3

#define PWM0_TIM_CH			TIM_CHANNEL_4
#define PWM1_TIM_CH			TIM_CHANNEL_1
#define PWM2_TIM_CH			TIM_CHANNEL_1
#define PWM3_TIM_CH			TIM_CHANNEL_2
#define PWM4_TIM_CH			TIM_CHANNEL_3
#define PWM5_TIM_CH			TIM_CHANNEL_4

#define PWM0_TIM_CCR		CCR4
#define PWM1_TIM_CCR		CCR1
#define PWM2_TIM_CCR		CCR1
#define PWM3_TIM_CCR		CCR2
#define PWM4_TIM_CCR		CCR3
#define PWM5_TIM_CCR		CCR4

#define FSW_on(n)		((n != 0) ? ((n != 1) ? (((n != 2) ? (((n != 3) ? (((n != 4) ? FSW5_on : FSW4_on)) : FSW3_on)) : FSW2_on)) : FSW1_on) : FSW0_on)
#define FSW_off(n)		((n != 0) ? ((n != 1) ? (((n != 2) ? (((n != 3) ? (((n != 4) ? FSW5_off : FSW4_off)) : FSW3_off)) : FSW2_off)) : FSW1_off) : FSW0_off)

/* ========================================================================== */
/*  시스템 런타임 상태 (전역 g_sys)                                            */
/* ========================================================================== */
/*
 * 제어 루프 스케줄링과 시스템 전반의 진단 카운터를 담는 전역 구조체.
 * 추후 uptime, boot 시각, CAN bus-off 카운트, UART overflow 등 시스템
 * 수준 통계가 추가될 가능성이 있어 확장 여지를 둔 설계.
 *
 * [ctrl_tick]
 *   TIM ISR(100Hz)에서 증가, 메인 루프가 atomic swap으로 0 리셋.
 *   volatile 필수 — ISR과 메인 루프가 교차 접근.
 *   uint32_t: ARM single-cycle store이지만 `if(>0){=0;}` 2줄 RMW는
 *   race 가능 → __disable_irq() 짧은 critical section으로 보호.
 *
 * [ctrl_overrun_count / ctrl_overrun_max]
 *   메인 루프가 10ms 이상 걸려 여러 틱이 쌓인 경우 (pending-1)을
 *   ctrl_overrun_count에 누적, 관측된 최대 pending을 ctrl_overrun_max에
 *   기록. catch-up 실행은 하지 않음(PID dt 왜곡/I2C burst 회피).
 *   메인 루프 단독 접근 → volatile/critical section 불필요.
 *   정상 상태에선 count=0, max≤1. D 디버그 명령으로 가시화.
 *
 * [ctrl_loop_hz / ctrl_loop_period_ms] — 제어 루프 타이밍
 *   TIM 실제 설정(Prescaler, Period)과 HCLK로부터 tim.c의 USER CODE
 *   블록에서 런타임 계산되어 저장. 매직 넘버 하드코드를 피하기 위함.
 *   main.c의 SMA_Init()에 g_sys.ctrl_loop_period_ms를 전달하는 식으로
 *   TIM 설정 변경 시 자동 반영.
 *   가정: APB2 divider ≤ 2 → TIM kernel = HCLK (현 SystemClock_Config
 *   RCC_APB2_DIV2 기준). 가정 깨지면 tim.c 계산식 재검토 필요.
 *
 * 부팅 시 BSS 0 초기화 → ctrl_tick/overrun_*는 0부터 시작,
 * ctrl_loop_*는 MX_TIM_Init() 호출 직후 tim.c USER CODE에서 세팅됨.
 */
typedef struct
{
    volatile uint32_t ctrl_tick;            /**< (legacy) TIM ISR 증가 — 본 phase에서 미사용, 후속 정리 */
    uint32_t          ctrl_overrun_count;   /**< (legacy) */
    uint32_t          ctrl_overrun_max;     /**< (legacy) */
    uint32_t          ctrl_loop_hz;         /**< (legacy) */
    uint32_t          ctrl_loop_period_ms;  /**< (legacy) */
    uint8_t           state_command;        /**< (legacy) */

    /* ── 두 박자 cooperative 처리용 (Phase 1 신규) ──
     * fast_tick : TIM4 ISR(~250Hz) 증가, main loop atomic-swap 리셋
     * slow_tick : TIM5 ISR(~10Hz)  증가, main loop atomic-swap 리셋
     * volatile 필수 — ISR과 main loop 교차 접근.
     * `if (>0) { =0; }` 2줄 RMW는 race 가능 → __disable_irq() 짧은 critical section.
     *
     * overrun_count / overrun_max : main loop가 한 주기 이상 걸려 여러 틱이
     * 쌓인 경우 (pending-1)을 누적, 관측된 최대 pending을 기록. catch-up
     * 실행은 하지 않음(PID dt 왜곡 / I2C burst 회피). 정상 상태에선 count=0,
     * max≤1. main loop 단독 접근 → volatile 불필요.
     */
    volatile uint32_t fast_tick;
    volatile uint32_t slow_tick;
    uint32_t          fast_overrun_count;
    uint32_t          fast_overrun_max;
    uint32_t          slow_overrun_count;
    uint32_t          slow_overrun_max;
} SystemState_t;

extern SystemState_t g_sys;

/* ── Safety mode 레벨 ── */
#define SAFETY_MODE_NORMAL      0
#define SAFETY_MODE_WARN        1
#define SAFETY_MODE_CRITICAL    2
#define SAFETY_MODE_SENSOR_ERR  3

/* ── Safety 온도 임계값 ── */
#define SAFETY_TEMP_WARN        80.0f
#define SAFETY_TEMP_WARN_HYST   75.0f   /* warn → normal 복구 히스테리시스 */
#define SAFETY_TEMP_CRITICAL    120.0f
#define SAFETY_TEMP_CRIT_HYST   30.0f   /* critical → normal 복구 히스테리시스 */
#define SAFETY_TEMP_SENSOR_HI   200.0f
#define SAFETY_TEMP_SENSOR_LO   -50.0f

/* ── Fan 송신 코드 (CMD 0x02 fan[i] 매립) ── */
#define FAN_CODE_OFF            0
#define FAN_CODE_ON             1
#define FAN_CODE_WARN           17
#define FAN_CODE_SENSOR_ERR     49

#endif

