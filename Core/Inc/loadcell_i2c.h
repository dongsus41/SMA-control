#ifndef __LOADCELL_I2C_H
#define __LOADCELL_I2C_H

#include "main.h"

#define LOADCELL_I2C_ADDR    (0x08 << 1)
#define LOADCELL_DATA_SIZE   8

typedef struct {
    float displacement;      // 엔코더 변위 (mm)
    float force;             // 로드셀 힘 (g 또는 N)
    uint32_t timestamp;      // 마지막 수신 시각
    uint8_t valid;           // 데이터 유효 여부
    uint8_t error_count;     // 연속 에러 카운트
} LoadCell_Data_TypeDef;

HAL_StatusTypeDef LoadCell_Read(I2C_HandleTypeDef *hi2c, LoadCell_Data_TypeDef *data);
void LoadCell_Init(LoadCell_Data_TypeDef *data);

#endif