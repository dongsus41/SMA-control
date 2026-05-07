#include "main.h"

extern FDCAN_HandleTypeDef hfdcan3;

extern UART_HandleTypeDef huart3;

static uint32_t time_tick;

static void FDCAN_Config_RX(FDCAN_HandleTypeDef* hfdcan, uint32_t *add_list, uint16_t add_list_size);
static void FDCAN_Config_RX_range(FDCAN_HandleTypeDef* hfdcan, uint32_t add_range1, uint32_t add_range2);
static void FDCAN_Config_TX_TDC(FDCAN_HandleTypeDef *hfdcan);

//---------------------------------------------------------------------------------------------------------------------------------------------
void FDCAN_Init(void)
{
    uint32_t rx_id_can3[] = {CAN3_RXID_COMMAND, CAN3_RXID_GAIN_UPDATE};
    FDCAN_Config_RX(&hfdcan3, rx_id_can3, CAN3_RXADD_N);
	
//    FDCAN_Config_TX(&can1_tx_header, 0x100, FDCAN_DLC_BYTES_64);
	FDCAN_Config_TX_TDC(&hfdcan3);

    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN_Config_TX(FDCAN_TxHeaderTypeDef *fdcan_tx_header, uint32_t tx_add, uint32_t tx_byte)
{
	/* Prepare Tx Header */
	fdcan_tx_header->Identifier = tx_add;
	fdcan_tx_header->IdType = FDCAN_STANDARD_ID;
	fdcan_tx_header->TxFrameType = FDCAN_DATA_FRAME;
	fdcan_tx_header->DataLength = tx_byte;
	fdcan_tx_header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	fdcan_tx_header->BitRateSwitch = FDCAN_BRS_OFF;
	fdcan_tx_header->FDFormat = FDCAN_FD_CAN;
	fdcan_tx_header->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	fdcan_tx_header->MessageMarker = 0;
}

static void FDCAN_Config_RX(FDCAN_HandleTypeDef* hfdcan, uint32_t *add_list, uint16_t add_list_size)
{
    FDCAN_FilterTypeDef sFilterConfig;

	for (uint8_t i=0; i<add_list_size; i++)
	{
	    sFilterConfig.IdType = FDCAN_STANDARD_ID;
	    sFilterConfig.FilterIndex = i;
	    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	    sFilterConfig.FilterID1 = add_list[i];
	    sFilterConfig.FilterID2 = 0x7FF;
	    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
		{
	        Error_Handler();
	    }
	}

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
        Error_Handler();
    }
}

static void FDCAN_Config_RX_range(FDCAN_HandleTypeDef* hfdcan, uint32_t add_range1, uint32_t add_range2)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = add_range1;
    sFilterConfig.FilterID2 = add_range2;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
        Error_Handler();
    }
}

static void FDCAN_Config_TX_TDC(FDCAN_HandleTypeDef *hfdcan)
{
/* Configure and enable Tx Delay Compensation, required for BRS mode.
TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
TdcFilter default recommended value: 0 */
	uint32_t tdcOffset = hfdcan->Init.DataPrescaler * hfdcan->Init.DataTimeSeg1;
	if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, tdcOffset, 0) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK)
	{
		Error_Handler();
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
#define HAL_UART_MAX_DELAY 10

//int _write(int file, char *ptr, int len)
//{
//    HAL_UART_Transmit(SERIAL_HADD, (uint8_t *)ptr, len, HAL_UART_MAX_DELAY);
//    return len;
//}
//
//int _read(int file, char *ptr, int len)
//{
//    int DataIdx = 0;
//    uint8_t receivedChar;
//
//    for (DataIdx = 0; DataIdx < len; DataIdx++)
//    {
//        HAL_UART_Receive(SERIAL_HADD, &receivedChar, 1, HAL_MAX_DELAY);
//        ptr[DataIdx] = receivedChar;
//        if (receivedChar == '\n' || receivedChar == '\r') // Enter Ű ó��
//        {
//            ptr[DataIdx] = '\0';
//			HAL_UART_Transmit(SERIAL_HADD, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
//            return DataIdx;
//        }
//    }
//    return DataIdx;
//}

int __io_putchar(int ch)
{
	if(HAL_UART_Transmit(SERIAL_HADD, (uint8_t *)&ch, 1, 10) != HAL_OK)
		return -1;
	return ch;
}

int __io_getchar(void)
{
	char data[4];
	uint8_t ch, len = 1;

	while(HAL_UART_Receive(SERIAL_HADD, &ch, 1, 10) != HAL_OK);

	memset(data, 0x00, 4);
	switch(ch)
	{
		case '\r':
		case '\n':
			len = 2;
			sprintf(data, "\r\n");
			break;

		case '\b':
		case 0x7F:
			len = 3;
			sprintf(data, "\b \b");
			break;

		default:
			data[0] = ch;
			break;
	}
	HAL_UART_Transmit(SERIAL_HADD, (uint8_t *)data, len, 10);
	return ch;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//uint32_t start = DWT->CYCCNT;
//uint32_t cycles = DWT->CYCCNT - start;
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // DWT Ȱ��ȭ
    DWT->CYCCNT = 0; // ����Ŭ ī���� �ʱ�ȭ
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // ī���� ����
}

void Timer_Start(void)
{
    time_tick = HAL_GetTick();
//    printf("Timer started at %lu ms\r\n", time_tick);
}

void Timer_Stop(void)
{
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms = current_tick - time_tick;
//	printf("Timer stopped at %lu ms\r\n", current_tick);
    printf("Elapsed time: %lu ms\r\n", elapsed_ms);
}

