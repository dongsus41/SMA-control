#include "uart_protocol.h"
#include <stdio.h>
#include "loadcell_i2c.h"
#include "force_control.h"

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static UART_HandleTypeDef *g_huart = NULL;
static UartRxContext g_rx_ctx;

extern System_typedef system;
extern PID_Manager_typedef pid;
extern I2C_HandleTypeDef hi2c2;

/* Forward declarations for functions defined in sma_actuator.c */
extern void Manual_Control(uint8_t ch);

/*******************************************************************************
 * CRC-8 (polynomial 0x07, init 0x00)
 ******************************************************************************/
// 통신 중 데이터가 깨지지 않았는지 확인
uint8_t UartComm_CalcCRC8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/*******************************************************************************
 * Build & Send a frame
 ******************************************************************************/
static void UartComm_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[UART_MAX_FRAME_SIZE];
    uint16_t idx = 0;

    frame[idx++] = UART_STX;
    frame[idx++] = cmd;
    frame[idx++] = len;

    if (payload && len > 0)
    {
        memcpy(&frame[idx], payload, len);
        idx += len;
    }

    frame[idx] = UartComm_CalcCRC8(&frame[1], 2 + len);
    idx++;

    HAL_UART_Transmit(g_huart, frame, idx, 10);
}

/*******************************************************************************
 * Send State Report (MCU -> PC)
 *   Layout: pwm[6] + fan[6] + temp[6*uint16] = 24 bytes
 ******************************************************************************/
void UartComm_SendState(void)
{
    UartComm_SendFrame(CMD_STATE,
                       system.buf_fdcan_tx.uint8,
                       STATE_PAYLOAD_SIZE);
}

/*******************************************************************************
 * Handle Control Command (PC -> MCU)
 *   Layout: pwm[6] + fan[6] + enable_pid[6] + target_temp[6*uint16] = 30 bytes
 *   This is the exact same logic as your FDCAN CAN3_RXID_COMMAND handler
 ******************************************************************************/
static void UartComm_HandleControl(const uint8_t *data, uint8_t len)
{
    if (len != CTRL_PAYLOAD_SIZE)
        return;

    if (system.state_level != SYSTEM_GO)
        return;

    memcpy(&system.ctrl_param_now, data, RX_BYTE_CTRL_PARAM);

    for (uint8_t i = 0; i < CTRL_CH; i++)
    {
        if (pid.enable_pid[i] != system.ctrl_param_now.enable_pid[i])
        {
            pid.enable_pid[i] = system.ctrl_param_now.enable_pid[i];

            if (pid.enable_pid[i])
            {
                pid.params[i].u_old = 0.0f;
                pid.params[i].last_error = 0.0f;
                pid.params[i].safety_mode = 0;

                float new_target = (float)system.ctrl_param_now.target_temp[i] / 4.0f;
                pid.params[i].setpoint = new_target;

                //printf("CH %u PID enabled, Target temp: %.2f\r\n", i, pid.params[i].setpoint);
            }
            else
            {
                //printf("CH %u PID disabled\r\n", i);
                Manual_Control(i);

                if (pid.params[i].safety_mode > 0)
                {
                    //printf("CH %u Safety mode reset by PID disable command\r\n", i);
                    pid.params[i].safety_mode = 0;
                    Update_Fan_Status(i);
                }
            }
        }
        else if (pid.enable_pid[i] &&
                 ((float)system.ctrl_param_now.target_temp[i] / 4.0f != pid.params[i].setpoint))
        {
            float new_target = (float)system.ctrl_param_now.target_temp[i] / 4.0f;
            pid.params[i].setpoint = new_target;

            if (pid.params[i].setpoint > pid.params[i].max_temp)
            {
                pid.params[i].setpoint = pid.params[i].max_temp;
            }
            //printf("CH %u Target temp updated: %.2f\r\n", i, pid.params[i].setpoint);
        }

        if (!pid.enable_pid[i])
        {
            Manual_Control(i);
        }
    }

    LED1_toggle;
    system.ctrl_param_save = system.ctrl_param_now;
}

/*******************************************************************************
 * Handle Gain Update (PC -> MCU)
 *   Layout: kp(float) + ki(float) + kd(float) + channel(uint8) = 13 bytes
 ******************************************************************************/
static void UartComm_HandleGainUpdate(const uint8_t *data, uint8_t len)
{
    if (len != GAIN_PAYLOAD_SIZE)
        return;

    memcpy(&pid.buf_fdcan_pid_tuning.uint8, data, RX_BYTE_PID_TUNING);

    uint8_t ch = pid.buf_fdcan_pid_tuning.struc.channel;
    if (ch < CTRL_CH)
    {
        pid.params[ch].lambda = pid.buf_fdcan_pid_tuning.struc.kp;
        pid.params[ch].alpha  = pid.buf_fdcan_pid_tuning.struc.ki;
        pid.params[ch].gain   = pid.buf_fdcan_pid_tuning.struc.kd;

        printf("PID Gains updated for CH %u: Kp=%.2f, Ki=%.4f, Kd=%.4f\r\n",
               ch, pid.params[ch].lambda, pid.params[ch].alpha, pid.params[ch].gain);

        LED2_toggle;
    }
    else
    {
        printf("Error: Invalid channel number %u\r\n", ch);
    }
}

/*******************************************************************************
 * Handle Force Control Command (PC -> MCU)
 *   Layout: channel(1) + enable(1) + target_force(float) = 6 bytes
 ******************************************************************************/
static void UartComm_HandleForceControl(const uint8_t *data, uint8_t len)
{
    if (len != FORCE_CTRL_PAYLOAD_SIZE)
        return;

    uint8_t channel = data[0];
    uint8_t enable  = data[1];
    float target_force;
    memcpy(&target_force, &data[2], 4);

    if (channel >= CTRL_CH)
    {
        printf("Error: Invalid force channel %u\r\n", channel);
        return;
    }

    if (enable)
    {
        ForceControl_SetTarget(target_force);
        ForceControl_Enable(channel);
    }
    else
    {
        ForceControl_Disable();
    }
}

/*******************************************************************************
 * Handle I2C Test (PC -> MCU)
 *   No payload — just attempt I2C read and printf result
 ******************************************************************************/
static void UartComm_HandleI2CTest(const uint8_t *data, uint8_t len)
{
    LoadCell_Data_TypeDef test_data;
    LoadCell_Init(&test_data);

    HAL_StatusTypeDef ret = LoadCell_Read(&hi2c2, &test_data);
    if (ret == HAL_OK && test_data.valid)
    {
        printf("I2C OK: disp=%.3f mm, force=%.3f g\r\n",
               test_data.displacement, test_data.force);
    }
    else
    {
        printf("I2C FAIL: HAL_Status=%d\r\n", ret);
    }
}

/*******************************************************************************
 * Send Force State Report (MCU -> PC)
 *   Layout: mode(1) + channel(1) + force(float) + displacement(float) = 10 bytes
 ******************************************************************************/
void UartComm_SendForceState(void)
{
    uint8_t payload[FORCE_STATE_PAYLOAD_SIZE];
    payload[0] = (uint8_t)force_ctrl.mode;
    payload[1] = force_ctrl.active_channel;
    memcpy(&payload[2], (void*)&force_ctrl.loadcell.force, 4);
    memcpy(&payload[6], (void*)&force_ctrl.loadcell.displacement, 4);
    UartComm_SendFrame(CMD_FORCE_STATE, payload, FORCE_STATE_PAYLOAD_SIZE);
}

/*******************************************************************************
 * Process complete frame
 ******************************************************************************/
static void UartComm_HandleFrame(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    switch (cmd)
    {
    case CMD_CONTROL:
        UartComm_HandleControl(data, len);
        break;
    case CMD_GAIN_UPDATE:
        UartComm_HandleGainUpdate(data, len);
        break;
    case CMD_FORCE_CONTROL:
        UartComm_HandleForceControl(data, len);
        break;
    case CMD_I2C_TEST:
        UartComm_HandleI2CTest(data, len);
        break;
    default:
        break;
    }
}

/*******************************************************************************
 * Byte-by-byte RX State Machine
 ******************************************************************************/
void UartComm_ProcessRxByte(uint8_t byte)
{
    switch (g_rx_ctx.state)
    {
    case RX_WAIT_STX:
        if (byte == UART_STX)
            g_rx_ctx.state = RX_WAIT_CMD;
        break;

    case RX_WAIT_CMD:
        g_rx_ctx.cmd = byte;
        g_rx_ctx.state = RX_WAIT_LEN;
        break;

    case RX_WAIT_LEN:
        g_rx_ctx.len = byte;
        g_rx_ctx.data_idx = 0;
        if (byte == 0)
            g_rx_ctx.state = RX_WAIT_CRC;
        else if (byte > UART_MAX_DATA_SIZE)
            g_rx_ctx.state = RX_WAIT_STX;
        else
            g_rx_ctx.state = RX_WAIT_DATA;
        break;

    case RX_WAIT_DATA:
        g_rx_ctx.data[g_rx_ctx.data_idx++] = byte;
        if (g_rx_ctx.data_idx >= g_rx_ctx.len)
            g_rx_ctx.state = RX_WAIT_CRC;
        break;

    case RX_WAIT_CRC:
    {
        uint8_t crc_buf[2 + UART_MAX_DATA_SIZE];
        crc_buf[0] = g_rx_ctx.cmd;
        crc_buf[1] = g_rx_ctx.len;
        memcpy(&crc_buf[2], g_rx_ctx.data, g_rx_ctx.len);

        uint8_t calc_crc = UartComm_CalcCRC8(crc_buf, 2 + g_rx_ctx.len);

        if (calc_crc == byte)
        {
            UartComm_HandleFrame(g_rx_ctx.cmd, g_rx_ctx.data, g_rx_ctx.len);
        }

        g_rx_ctx.state = RX_WAIT_STX;
        break;
    }

    default:
        g_rx_ctx.state = RX_WAIT_STX;
        break;
    }
}

/*******************************************************************************
 * Init
 ******************************************************************************/
void UartComm_Init(UART_HandleTypeDef *huart)
{
    g_huart = huart;
    memset(&g_rx_ctx, 0, sizeof(g_rx_ctx));
    g_rx_ctx.state = RX_WAIT_STX;

    HAL_UART_Receive_IT(g_huart, &g_rx_ctx.rx_byte, 1);
}

/*******************************************************************************
 * HAL UART RX Callback
 ******************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == g_huart->Instance)
    {
        UartComm_ProcessRxByte(g_rx_ctx.rx_byte);
        HAL_UART_Receive_IT(g_huart, &g_rx_ctx.rx_byte, 1);
    }
}
