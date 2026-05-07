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
 *   0x01  Control Command   PC -> MCU   30 bytes  (Ctrl_Param_typedef)
 *   0x02  State Report      MCU -> PC   24 bytes  (Buf_FDCAN_Tx_typedef)
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
#define CMD_FORCE_CONTROL     0X04
#define CMD_FORCE_STATE       0X05
#define CMD_I2C_TEST          0X06

#define CTRL_PAYLOAD_SIZE     RX_BYTE_CTRL_PARAM   /* 30 */
#define STATE_PAYLOAD_SIZE    TX_BYTE_FDCAN         /* 24 */
#define GAIN_PAYLOAD_SIZE     RX_BYTE_PID_TUNING    /* 13 */
#define FORCE_CTRL_PAYLOAD_SIZE    6
#define FORCE_STATE_PAYLOAD_SIZE   10

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
