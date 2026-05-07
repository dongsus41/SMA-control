#ifndef __FDCAN_H
#define __FDCAN_H

void FDCAN_Init(void);
void FDCAN_Config_TX(FDCAN_TxHeaderTypeDef *fdcan_tx_header, uint32_t tx_add, uint32_t tx_byte);

//int _write(int file, char *ptr, int len);
//int _read(int file, char *ptr, int len);
int __io_putchar(int ch);
int __io_getchar(void);

void DWT_Init(void);
void Timer_Start(void);
void Timer_Stop(void);

#endif /* __FDCAN_H */
