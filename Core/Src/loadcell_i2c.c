#include "loadcell_i2c.h"
#include <string.h>

#define LOADCELL_MAX_ERROR_COUNT  10

void LoadCell_Init(LoadCell_Data_TypeDef *data)
{
    data->displacement = 0.0f;
    data->force = 0.0f;
    data->timestamp = 0;
    data->valid = 0;
    data->error_count = 0;
}

HAL_StatusTypeDef LoadCell_Read(I2C_HandleTypeDef *hi2c, LoadCell_Data_TypeDef *data)
{
    uint8_t rx_buf[LOADCELL_DATA_SIZE];

    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(
        hi2c, LOADCELL_I2C_ADDR, rx_buf, LOADCELL_DATA_SIZE, 50);

    if (ret == HAL_OK)
    {
        memcpy(&data->displacement, &rx_buf[0], 4); 
        memcpy(&data->force, &rx_buf[4], 4);
        data->timestamp = HAL_GetTick();
        data->valid = 1;
        data->error_count = 0;
    }
    else
    {
        data->error_count++;
        if (data->error_count >= LOADCELL_MAX_ERROR_COUNT)
        {
            data->valid = 0;
        }
    }
    return ret;
}

/* ═══════════════ Phase 5: Ring Buffer 평균 ═══════════════ */

#define LOADCELL_AVG_SIZE     8U      /* 8샘플 평균 → 슬레이브 80Hz × 8 = ~10Hz force PID 박자 */
#define LOADCELL_POLL_CAP_MS  12U     /* HX711 80Hz (12.5ms)와 거의 동일 */

static float    s_force_buf[LOADCELL_AVG_SIZE];
static uint8_t  s_buf_idx     = 0;    /* 다음 write 위치 (0~7 wrap) */
static uint8_t  s_buf_filled  = 0;    /* 현재 채워진 샘플 수 (max 8) */
static float    s_latest_disp = 0.0f;
static uint32_t s_last_read_ms  = 0;  /* 마지막 시도(성공/실패 무관) */
static uint32_t s_last_valid_ms = 0;  /* 마지막 성공 read */

void LoadCell_Update(I2C_HandleTypeDef *hi2c, uint32_t now_ms)
{
    /* 12ms 캡 — 슬레이브 갱신 주기보다 빨리 폴링 안 함 */
    if ((uint32_t)(now_ms - s_last_read_ms) < LOADCELL_POLL_CAP_MS) return;
    s_last_read_ms = now_ms;

    LoadCell_Data_TypeDef data;
    LoadCell_Init(&data);
    if (LoadCell_Read(hi2c, &data) == HAL_OK && data.valid)
    {
        s_force_buf[s_buf_idx] = data.force;
        s_buf_idx = (uint8_t)((s_buf_idx + 1U) % LOADCELL_AVG_SIZE);
        if (s_buf_filled < LOADCELL_AVG_SIZE) s_buf_filled++;
        s_latest_disp   = data.displacement;
        s_last_valid_ms = now_ms;
    }
    /* 실패 시 ring buffer 변경 없음 — 이전 평균 유지 */
}

float LoadCell_GetAverage(void)
{
    if (s_buf_filled == 0U) return 0.0f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < s_buf_filled; i++) {
        sum += s_force_buf[i];
    }
    return sum / (float)s_buf_filled;
}

float LoadCell_GetLatestDisp(void)
{
    return s_latest_disp;
}

uint8_t LoadCell_IsStale(uint32_t now_ms, uint32_t timeout_ms)
{
    return ((uint32_t)(now_ms - s_last_valid_ms) > timeout_ms) ? 1U : 0U;
}
