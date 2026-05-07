#include "uart_protocol.h"
#include <stdio.h>
#include "loadcell_i2c.h"
#include "force_control.h"
#include "controller.h"

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static UART_HandleTypeDef *g_huart = NULL;
static UartRxContext g_rx_ctx;

/* ── RX Ring Buffer (SPSC lock-free) ──
 * Producer : HAL_UART_RxCpltCallback (ISR)
 * Consumer : UartComm_Process() (main loop)
 *
 * 동기화 모델:
 *   head : ISR(push)에서만 갱신, main loop(pop)에서 읽음
 *   tail : main loop(pop)에서만 갱신, ISR(push)에서 읽음
 *   둘 다 1바이트 atomic write이므로 lock/critical section 불필요.
 *
 * 크기 256 = uint8_t 자연 wrap-around → mod 연산 불필요.
 * 1 entry는 full/empty 구분 위해 사용 못하므로 효과적 용량 255 byte.
 * UART_MAX_FRAME_SIZE(254 byte)보다 약간 큼 — main loop가 한 frame 분량
 * 시간(115200 baud에서 ~22ms) 안에 한 번 돌면 overflow 안 남.
 *
 * dropped_count : FIFO full 시 폐기된 byte 누적. 디버거로 main loop 지연 진단.
 */
#define UART_RX_FIFO_SIZE   256U

typedef struct {
    uint8_t           buf[UART_RX_FIFO_SIZE];
    volatile uint8_t  head;
    volatile uint8_t  tail;
    volatile uint16_t dropped_count;
} UartRxFifo_t;

static UartRxFifo_t g_rx_fifo;

/* ISR-callable: byte 1개 push. FIFO full 시 byte 폐기 + counter 증가. */
static inline void uart_rx_fifo_push(uint8_t b)
{
    uint8_t next = (uint8_t)(g_rx_fifo.head + 1U);
    if (next == g_rx_fifo.tail) {
        g_rx_fifo.dropped_count++;
        return;
    }
    g_rx_fifo.buf[g_rx_fifo.head] = b;
    g_rx_fifo.head = next;
}

/* main-callable: byte 1개 pop. FIFO empty 시 0 반환, 데이터 있으면 1 반환. */
static inline uint8_t uart_rx_fifo_pop(uint8_t *out)
{
    if (g_rx_fifo.head == g_rx_fifo.tail) {
        return 0;
    }
    *out = g_rx_fifo.buf[g_rx_fifo.tail];
    g_rx_fifo.tail = (uint8_t)(g_rx_fifo.tail + 1U);
    return 1;
}

extern System_typedef       system;     /* state_level 용 (Phase 6에서 정리) */
extern ForceControl_TypeDef force_ctrl; /* SendForceState transitional */
extern I2C_HandleTypeDef    hi2c2;      /* I2C Test (CMD 0x06) 용 */

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
 * Send State Report (MCU -> PC) — CMD 0x02, 24 byte
 *   Phase 4: g_state buffer 사용 (Actuator_Apply가 채움).
 *   레이아웃: pwm[6] + fan[6] + temp[6*uint16] (state_buf_t)
 ******************************************************************************/
void UartComm_SendState(void)
{
    UartComm_SendFrame(CMD_STATE,
                       (const uint8_t*)&g_state,
                       STATE_PAYLOAD_SIZE);
}

/*******************************************************************************
 * Handle Control Command (PC -> MCU) — CMD 0x01, 30 byte
 *   Phase 4 새 페이로드:
 *     mode[6]        : ch_ctrl_e (offset 0)
 *     manual_pwm[6]  : 0~100 (offset 6)
 *     manual_fan[6]  : 0/1 (offset 12)
 *     target[6×u16]  : TEMP 0.25°C/LSB / FORCE 0.1g/LSB (offset 18, little-endian)
 *
 *   FORCE 동시 1채널 제약: mode[]에 CH_FORCE가 2개 이상이면 프레임 거부.
 ******************************************************************************/
static void UartComm_HandleControl(const uint8_t *data, uint8_t len)
{
    if (len != CTRL_PAYLOAD_SIZE)  /* 30 */
        return;

    if (system.state_level != SYSTEM_GO)
        return;

    /* FORCE 동시 1채널 제약 검증 */
    uint8_t force_count = 0;
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        if (data[i] == CH_FORCE) force_count++;
    }
    if (force_count > 1) {
        printf("WARN: CMD_CONTROL rejected — multiple CH_FORCE channels (%u)\r\n", force_count);
        return;
    }

    /* mode 변경 시 PID 상태 reset */
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        if (g_cmd.mode[i] != data[i]) {
            g_ctrl.temp_params[i].u_old      = 0.0f;
            g_ctrl.temp_params[i].last_error = 0.0f;
            g_ctrl.safety_mode[i]            = 0;
        }
    }

    /* 페이로드 디코드 → g_cmd */
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        g_cmd.mode[i]       = data[0  + i];
        g_cmd.manual_pwm[i] = data[6  + i];
        g_cmd.manual_fan[i] = data[12 + i];
        g_cmd.target[i]     = (uint16_t)data[18 + i*2] | ((uint16_t)data[19 + i*2] << 8);
    }

    LED1_toggle;
}

/*******************************************************************************
 * Handle Gain Update (PC -> MCU) — CMD 0x03, 13 byte
 *   Phase 4: g_ctrl.temp_params 향함.
 *   Layout: kp(float) + ki(float) + kd(float) + channel(uint8) = 13 bytes
 ******************************************************************************/
static void UartComm_HandleGainUpdate(const uint8_t *data, uint8_t len)
{
    if (len != GAIN_PAYLOAD_SIZE)
        return;

    float kp, ki, kd;
    memcpy(&kp, &data[0], 4);
    memcpy(&ki, &data[4], 4);
    memcpy(&kd, &data[8], 4);
    uint8_t ch = data[12];

    if (ch >= CTRL_CH) {
        printf("Error: Invalid channel number %u\r\n", ch);
        return;
    }

    /* PID_Param_TypeDef는 lambda/alpha/gain으로 저장 */
    g_ctrl.temp_params[ch].lambda = kp;
    g_ctrl.temp_params[ch].alpha  = ki;
    g_ctrl.temp_params[ch].gain   = kd;

    printf("PID Gains updated for CH %u: Kp=%.2f, Ki=%.4f, Kd=%.4f\r\n",
           ch, kp, ki, kd);
    LED2_toggle;
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
 * Send Force State Report (MCU -> PC) — CMD 0x05, 10 byte
 *   Phase 4: g_cmd.mode 참조하여 active force 채널 식별.
 *   동시 1채널 제약상 첫 CH_FORCE 채널만 송신.
 *   Layout: mode(1) + channel(1) + force(float32_LE) + displacement(float32_LE)
 ******************************************************************************/
void UartComm_SendForceState(void)
{
    int8_t force_ch = -1;
    for (uint8_t i = 0; i < CTRL_CH; i++) {
        if (g_cmd.mode[i] == CH_FORCE) {
            force_ch = (int8_t)i;
            break;
        }
    }
    if (force_ch < 0) return;

    uint8_t payload[FORCE_STATE_PAYLOAD_SIZE];
    payload[0] = CH_FORCE;
    payload[1] = (uint8_t)force_ch;
    /* force_ctrl.loadcell 캐시 사용 — Controller_Update가 read한 latest 값 (transitional) */
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
    /* CMD_FORCE_CONTROL (0x04) 폐기 — Phase 4 통합 (CMD 0x01 mode=CH_FORCE) */
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
static void UartComm_ProcessRxByte(uint8_t byte)
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
 * Process all pending RX bytes (main loop)
 *   FIFO에서 byte를 모두 꺼내 state machine에 공급.
 *   Frame 완성 시 UartComm_HandleFrame이 호출되어 PID/Manual/Force 명령 처리.
 *   Frame 처리에 시간이 걸려도 ISR은 영향 없음 (ring buffer가 흡수).
 ******************************************************************************/
void UartComm_Process(void)
{
    uint8_t byte;
    while (uart_rx_fifo_pop(&byte))
    {
        UartComm_ProcessRxByte(byte);
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
        /* ISR 슬림화 (Phase 2): byte를 ring buffer에 push만,
         * frame state machine은 main loop의 UartComm_Process()가 처리. */
        uart_rx_fifo_push(g_rx_ctx.rx_byte);
        HAL_UART_Receive_IT(g_huart, &g_rx_ctx.rx_byte, 1);
    }
}
