#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "main.h"
#include "temp_control.h"
#include <stdint.h>
#include <string.h>

/*******************************************************************************
 * UART Protocol (PC <-> MCU)  — replaces FDCAN3 for PC communication
 *
 * Frame:  [0xAA] [CMD] [LEN] [DATA...] [CRC8]
 *
 *   0x01  Control Command   PC -> MCU   30 bytes  (cmd_param_t)
 *   0x02  State Report      MCU -> PC   24 bytes  (state_buf_t)
 *   0x03  Gain Update       PC -> MCU   13 bytes  (Buf_FDCAN_PID_Tuning)
 *   0x04  Force Control     PC -> MCU    6 bytes  (channel + enable + target)
 *   0x05  Force State       MCU -> PC   10 bytes  (mode + ch + force + disp)
 *   0x06  I2C Test          PC -> MCU    0 bytes  (통신 테스트)   
 ******************************************************************************/

#define UART_STX              0xAA
#define UART_MAX_DATA_SIZE    250
#define UART_MAX_FRAME_SIZE   (3 + UART_MAX_DATA_SIZE + 1)

#define CMD_CONTROL           0x01
#define CMD_STATE             0x02
#define CMD_GAIN_UPDATE       0x03
/* CMD_FORCE_CONTROL (0x04) Phase 4 폐기 — CMD 0x01 mode=CH_FORCE로 통합 */
#define CMD_FORCE_STATE       0X05
#define CMD_I2C_TEST          0X06

#define CTRL_PAYLOAD_SIZE          30   /* mode[6]+manual_pwm[6]+manual_fan[6]+target[6×u16] */
#define STATE_PAYLOAD_SIZE         24   /* pwm[6]+fan[6]+temp[6×u16] */
#define GAIN_PAYLOAD_SIZE          13   /* kp+ki+kd(float×3)+channel(u8) */
#define FORCE_STATE_PAYLOAD_SIZE   10   /* mode+ch(u8×2)+force+disp(float×2) */

typedef enum {
    RX_WAIT_STX,
    RX_WAIT_CMD,
    RX_WAIT_LEN,
    RX_WAIT_DATA,
    RX_WAIT_CRC
} UartRxState;

typedef struct {
    UartRxState state;
    uint8_t     cmd;
    uint8_t     len;
    uint8_t     data[UART_MAX_DATA_SIZE];
    uint8_t     data_idx;
    uint8_t     rx_byte;
} UartRxContext;

void    UartComm_Init(UART_HandleTypeDef *huart);
void    UartComm_Process(void);          /* main loop에서 주기 호출 — RX byte 소비 + frame 처리 */
void    UartComm_SendState(void);
void    UartComm_SendForceState(void);
uint8_t UartComm_CalcCRC8(const uint8_t *data, uint16_t len);

#endif /* UART_PROTOCOL_H */
