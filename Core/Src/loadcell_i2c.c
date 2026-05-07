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