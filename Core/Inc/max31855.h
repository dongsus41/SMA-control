#ifndef MAX31855_H_
#define MAX31855_H_

#define TMC_CH				6

#define TMC0_Enable			GPIO_WRITE_0(TMC_CS0_GPIO_Port, TMC_CS0_Pin)
#define TMC0_Disable		GPIO_WRITE_1(TMC_CS0_GPIO_Port, TMC_CS0_Pin)
#define TMC1_Enable			GPIO_WRITE_0(TMC_CS1_GPIO_Port, TMC_CS1_Pin)
#define TMC1_Disable		GPIO_WRITE_1(TMC_CS1_GPIO_Port, TMC_CS1_Pin)
#define TMC2_Enable			GPIO_WRITE_0(TMC_CS2_GPIO_Port, TMC_CS2_Pin)
#define TMC2_Disable		GPIO_WRITE_1(TMC_CS2_GPIO_Port, TMC_CS2_Pin)
#define TMC3_Enable			GPIO_WRITE_0(TMC_CS3_GPIO_Port, TMC_CS3_Pin)
#define TMC3_Disable		GPIO_WRITE_1(TMC_CS3_GPIO_Port, TMC_CS3_Pin)
#define TMC4_Enable			GPIO_WRITE_0(TMC_CS4_GPIO_Port, TMC_CS4_Pin)
#define TMC4_Disable		GPIO_WRITE_1(TMC_CS4_GPIO_Port, TMC_CS4_Pin)
#define TMC5_Enable			GPIO_WRITE_0(TMC_CS5_GPIO_Port, TMC_CS5_Pin)
#define TMC5_Disable		GPIO_WRITE_1(TMC_CS5_GPIO_Port, TMC_CS5_Pin)

#define TMC_SPI	 			SPI2
#define TMC_SPI_DMA			DMA1
#define TMC_SPI_DMA_RX_CH	LL_DMA_CHANNEL_2
//------------------------------------------------------------------------------------------------------------------------

typedef enum {
	TMC0 = 0,
	TMC1 = 1,
	TMC2 = 2,
	TMC3 = 3,
	TMC4 = 4,
	TMC5 = 5
}MAX31855_CH_typedef;

typedef struct {
	uint32_t oc				: 1;
	uint32_t scg			: 1;
	uint32_t scv			: 1;
	uint32_t reserved2		: 1;
	uint32_t temp_int12		: 12;
	uint32_t fault			: 1;
	uint32_t reserved1		: 1;
	uint32_t temp_ext14		: 14;
}MAX31855_SPI_typedef;

typedef union {
	uint8_t data_uint8[4];
	uint32_t data_uint32;
	MAX31855_SPI_typedef data_bf;
}MAX31855_SPI_Union_typedef;

typedef struct {
	MAX31855_SPI_Union_typedef spi_rx;
	MAX31855_SPI_Union_typedef spi_rx_swap;
	volatile uint8_t rx_byte_count;
	uint8_t rx_byte;
	uint16_t temp_ext14_raw[TMC_CH];
	uint16_t temp_int12_raw[TMC_CH];
	float temp_ext14[TMC_CH];
	float temp_int12[TMC_CH];
	double temp_correct[TMC_CH];
//	uint8_t flag_first_byte;
}MAX31855_typedef;
//------------------------------------------------------------------------------------------------------------------------

void LL_SPI_Setup (void);
//void LL_SPI_TMC_Rx (uint8_t tmc_ch, uint8_t* rx, uint8_t size);
void LL_SPI_TMC_Rx (uint8_t tmc_ch);
void TMC_Scan (uint8_t end_ch);
void TMC_IRQ_Handler (void);
void SPI_TransferError_Callback(void);
void MAX31855_Test (uint8_t ch);

double correctedCelsius(float temp_ext, float temp_int);
double correctedCelsius2(float temp_ext, float temp_int);

#endif /* MAX31855_H_ */

