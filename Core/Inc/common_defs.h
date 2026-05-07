#ifndef __COMMON_DEFS_H
#define __COMMON_DEFS_H

#define SERIAL_HADD					(&huart3)	//JETSON

#define CAN3_RXID_COMMAND 			0x400
#define CAN3_RXID_GAIN_UPDATE 		0x402
#define CAN3_RXADD_N				2

#define CAN3_TXID_STATE				0x401

#define N_MOTION					12
#define N_TIMEBIN					2
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

#endif

