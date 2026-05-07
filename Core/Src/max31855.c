#include <math.h>
#include "main.h"

extern System_typedef system;
MAX31855_typedef tmc;

//seungchan.2024.11.03 : INT
void LL_SPI_Setup (void)
{
	LL_SPI_SetRxFIFOThreshold(TMC_SPI, LL_SPI_RX_FIFO_TH_QUARTER);

	LL_SPI_EnableIT_RXNE(TMC_SPI);
	LL_SPI_EnableIT_ERR(TMC_SPI);
}

void LL_SPI_TMC_Rx (uint8_t ch)
{
	tmc.rx_byte = 4;

	switch (ch)
	{
		case TMC0:
			TMC0_Enable;
			break;
		case TMC1:
			TMC1_Enable;
			break;
		case TMC2:
			TMC2_Enable;
			break;
		case TMC3:
			TMC3_Enable;
			break;
		case TMC4:
			TMC4_Enable;
			break;
		case TMC5:
			TMC5_Enable;
			break;
	}
	
	tmc.rx_byte_count = 0;		//seungchan.2024.11.03 : INT
//	LL_SPI_EnableIT_RXNE(TMC_SPI);
//	LL_SPI_EnableIT_ERR(TMC_SPI);

	LL_SPI_Enable(TMC_SPI);
	while (LL_SPI_IsActiveFlag_BSY(TMC_SPI) == 1);
	
//	LL_SPI_DisableIT_RXNE(TMC_SPI);
//	LL_SPI_DisableIT_ERR(TMC_SPI);

	switch (ch)
	{
		case TMC0:
			TMC0_Disable;
			break;
		case TMC1:
			TMC1_Disable;
			break;
		case TMC2:
			TMC2_Disable;
			break;
		case TMC3:
			TMC3_Disable;
			break;
		case TMC4:
			TMC4_Disable;
			break;
		case TMC5:
			TMC5_Disable;
			break;
	}
	
	tmc.spi_rx_swap.data_uint32 = SWAP32(tmc.spi_rx.data_uint32);

//	tmc.temp_ext14_raw[ch] = (int16_t)((tmc.spi_rx_swap.data_uint32 & 0x3FFF << 18) >> 18);
//	tmc.temp_int12_raw[ch] = (int16_t)((tmc.spi_rx_swap.data_uint32 & 0xFFF << 4) >> 4);

//	tmc.temp_ext14[ch] = ((int32_t)((tmc.spi_rx_swap.data_uint32 & 0x3FFF << 18) >> 18))*0.25f;
//	tmc.temp_int12[ch] = ((int32_t)((tmc.spi_rx_swap.data_uint32 & 0xFFF << 4) >> 4))*0.0625f;

	tmc.temp_ext14_raw[ch] = (int16_t)tmc.spi_rx_swap.data_bf.temp_ext14;
	tmc.temp_int12_raw[ch] = (int16_t)tmc.spi_rx_swap.data_bf.temp_int12;
	
	tmc.temp_ext14[ch] = tmc.temp_ext14_raw[ch]*0.25f;
	tmc.temp_int12[ch] = tmc.temp_int12_raw[ch]*0.0625f;
	
//	tmc.temp_correct[ch] = correctedCelsius(tmc.temp_ext14[ch], tmc.temp_int12[ch]);

	system.buf_fdcan_tx.struc.temp[ch] = tmc.temp_ext14_raw[ch];
//	system.buf_fdcan_tx.struc.temp[ch] = tmc.temp_int12_raw[ch];
}

void TMC_Scan (uint8_t end_ch)
{
	for (uint8_t ch=0; ch<end_ch; ch++)
	{
		LL_SPI_TMC_Rx (ch);
	}
}

void TMC_IRQ_Handler (void)
{
	if (LL_SPI_IsActiveFlag_RXNE(SPI2))
	{
	  tmc.spi_rx.data_uint8[tmc.rx_byte_count] = LL_SPI_ReceiveData8(SPI2); 	  //seungchan.2024.11.03 : INT
	  tmc.rx_byte_count++;
	  if (tmc.rx_byte_count == tmc.rx_byte)
	  {
		  LL_SPI_Disable(TMC_SPI);
	  }
	}
	else if (LL_SPI_IsActiveFlag_OVR(SPI2))
	{
	  SPI_TransferError_Callback();
	}
}

void SPI_TransferError_Callback(void)
{
	LL_DMA_DisableChannel(TMC_SPI_DMA, TMC_SPI_DMA_RX_CH);
	LL_SPI_DisableIT_RXNE(SPI2);

	while (1)
	{
		LED2_toggle;
		HAL_Delay(100);
	}
}

void MAX31855_Test (uint8_t ch)
{
  	LL_SPI_TMC_Rx (ch);
//	tmc.temp_correct[ch] = correctedCelsius(tmc.temp_ext14[ch], tmc.temp_int12[ch]);

//	float temp_ext14 = ((int32_t)((tmc.spi_rx_swap & 0x3FFF << 18) >> 18))*0.25f;
//	float temp_int12 = ((int32_t)((tmc.spi_rx_swap & 0xFFF << 4) >> 4))*0.0625f;	
//	double temp_correct1 =  correctedCelsius(temp_ext14, temp_int12);
//	double temp_correct2 =  correctedCelsius(temp_ext14, temp_int12);

//	printf("%.4f %.4f %.4lf\r\n", tmc.temp_ext14[ch], tmc.temp_int12[ch], tmc.temp_correct[ch]);
//	printf("%.4lf\r\n", tmc.temp_correct[ch]);

	printf("%.4f %.4f %hu %hu\r\n", tmc.temp_ext14[ch], tmc.temp_int12[ch], tmc.temp_ext14_raw[ch], tmc.temp_int12_raw[ch]);
	
//	while(UART_DEBUG.gState != HAL_UART_STATE_READY);
//	if(HAL_UART_Transmit_DMA(UART_DEBUG_ADD,(uint8_t *)system.uart_char, uart_buf_len) != HAL_OK)
//		HAL_UART_ErrorCallback (UART_DEBUG_ADD);
	
	LED2_toggle;
	HAL_Delay(100);
}

//------------------------------------------------------------------------------------------------------------------------

//https://learn.adafruit.com/calibrating-sensors/maxim-31855-linearization

// corrected temperature reading for a K-type thermocouple
// allowing accurate readings over an extended range
// http://forums.adafruit.com/viewtopic.php?f=19&t=32086&p=372992#p372992
// assuming global: Adafruit_MAX31855 thermocouple(CLK, CS, DO);
double correctedCelsius(float temp_ext, float temp_int)
{
   
   // MAX31855 thermocouple voltage reading in mV
   float thermocoupleVoltage = (temp_ext - temp_int) * 0.041276;
   
   // MAX31855 cold junction voltage reading in mV
   float coldJunctionTemperature = temp_int;
   float coldJunctionVoltage = -0.176004136860E-01 +
      0.389212049750E-01  * coldJunctionTemperature +
      0.185587700320E-04  * pow(coldJunctionTemperature, 2.0) +
      -0.994575928740E-07 * pow(coldJunctionTemperature, 3.0) +
      0.318409457190E-09  * pow(coldJunctionTemperature, 4.0) +
      -0.560728448890E-12 * pow(coldJunctionTemperature, 5.0) +
      0.560750590590E-15  * pow(coldJunctionTemperature, 6.0) +
      -0.320207200030E-18 * pow(coldJunctionTemperature, 7.0) +
      0.971511471520E-22  * pow(coldJunctionTemperature, 8.0) +
      -0.121047212750E-25 * pow(coldJunctionTemperature, 9.0) +
      0.118597600000E+00  * exp(-0.118343200000E-03 * 
                           pow((coldJunctionTemperature-0.126968600000E+03), 2.0) 
                        );
                        
                        
   // cold junction voltage + thermocouple voltage         
   float voltageSum = thermocoupleVoltage + coldJunctionVoltage;
   
   // calculate corrected temperature reading based on coefficients for 3 different ranges   
   float b0, b1, b2, b3, b4, b5, b6, b7, b8, b9;
   if(thermocoupleVoltage < 0){
      b0 = 0.0000000E+00;
      b1 = 2.5173462E+01;
      b2 = -1.1662878E+00;
      b3 = -1.0833638E+00;
      b4 = -8.9773540E-01;
      b5 = -3.7342377E-01;
      b6 = -8.6632643E-02;
      b7 = -1.0450598E-02;
      b8 = -5.1920577E-04;
      b9 = 0.0000000E+00;
   }
   
   else if(thermocoupleVoltage < 20.644){
      b0 = 0.000000E+00;
      b1 = 2.508355E+01;
      b2 = 7.860106E-02;
      b3 = -2.503131E-01;
      b4 = 8.315270E-02;
      b5 = -1.228034E-02;
      b6 = 9.804036E-04;
      b7 = -4.413030E-05;
      b8 = 1.057734E-06;
      b9 = -1.052755E-08;
   }
   
   else if(thermocoupleVoltage < 54.886){
      b0 = -1.318058E+02;
      b1 = 4.830222E+01;
      b2 = -1.646031E+00;
      b3 = 5.464731E-02;
      b4 = -9.650715E-04;
      b5 = 8.802193E-06;
      b6 = -3.110810E-08;
      b7 = 0.000000E+00;
      b8 = 0.000000E+00;
      b9 = 0.000000E+00;
   }
   
   else {
      // TODO: handle error - out of range
      return 0;
   }
   
   return b0 + 
      b1 * voltageSum +
      b2 * pow(voltageSum, 2.0) +
      b3 * pow(voltageSum, 3.0) +
      b4 * pow(voltageSum, 4.0) +
      b5 * pow(voltageSum, 5.0) +
      b6 * pow(voltageSum, 6.0) +
      b7 * pow(voltageSum, 7.0) +
      b8 * pow(voltageSum, 8.0) +
      b9 * pow(voltageSum, 9.0);
}
//------------------------------------------------------------------------------------------------------------------------

double correctedCelsius2 (float temp_ext, float temp_int)
{
	int i = 0;
   double internalTemp = temp_int; // Read the internal temperature of the MAX31855.
   double rawTemp = temp_ext; // Read the temperature of the thermocouple. This temp is compensated for cold junction temperature.
   double thermocoupleVoltage= 0;
   double internalVoltage = 0;
   double correctedTemp = 0;
   // Steps 1 & 2. Subtract cold junction temperature from the raw thermocouple temperature.
   thermocoupleVoltage = (rawTemp - internalTemp)*0.041276;  // C * mv/C = mV

   // Step 3. Calculate the cold junction equivalent thermocouple voltage.

   if (internalTemp >= 0) { // For positive temperatures use appropriate NIST coefficients
	  // Coefficients and equations available from http://srdata.nist.gov/its90/download/type_k.tab

	  double c[] = {-0.176004136860E-01,  0.389212049750E-01,  0.185587700320E-04, -0.994575928740E-07,  0.318409457190E-09, -0.560728448890E-12,  0.560750590590E-15, -0.320207200030E-18,  0.971511471520E-22, -0.121047212750E-25};

	  // Count the the number of coefficients. There are 10 coefficients for positive temperatures (plus three exponential coefficients),
	  // but there are 11 coefficients for negative temperatures.
	  int cLength = sizeof(c) / sizeof(c[0]);

	  // Exponential coefficients. Only used for positive temperatures.
	  double a0 =  0.118597600000E+00;
	  double a1 = -0.118343200000E-03;
	  double a2 =  0.126968600000E+03;


	  // From NIST: E = sum(i=0 to n) c_i t^i + a0 exp(a1 (t - a2)^2), where E is the thermocouple voltage in mV and t is the temperature in degrees C.
	  // In this case, E is the cold junction equivalent thermocouple voltage.
	  // Alternative form: C0 + C1*internalTemp + C2*internalTemp^2 + C3*internalTemp^3 + ... + C10*internaltemp^10 + A0*e^(A1*(internalTemp - A2)^2)
	  // This loop sums up the c_i t^i components.
	  for (i = 0; i < cLength; i++) {
		 internalVoltage += c[i] * pow(internalTemp, i);
	  }
		 // This section adds the a0 exp(a1 (t - a2)^2) components.
		 internalVoltage += a0 * exp(a1 * pow((internalTemp - a2), 2));
   }
   else if (internalTemp < 0) { // for negative temperatures
	  double c[] = {0.000000000000E+00,  0.394501280250E-01,  0.236223735980E-04, -0.328589067840E-06, -0.499048287770E-08, -0.675090591730E-10, -0.574103274280E-12, -0.310888728940E-14, -0.104516093650E-16, -0.198892668780E-19, -0.163226974860E-22};
	  // Count the number of coefficients.
	  int cLength = sizeof(c) / sizeof(c[0]);

	  // Below 0 degrees Celsius, the NIST formula is simpler and has no exponential components: E = sum(i=0 to n) c_i t^i
	  for (i = 0; i < cLength; i++) {
		 internalVoltage += c[i] * pow(internalTemp, i) ;
	  }
   }

   // Step 4. Add the cold junction equivalent thermocouple voltage calculated in step 3 to the thermocouple voltage calculated in step 2.
   double totalVoltage = thermocoupleVoltage + internalVoltage;

   // Step 5. Use the result of step 4 and the NIST voltage-to-temperature (inverse) coefficients to calculate the cold junction compensated, linearized temperature value.
   // The equation is in the form correctedTemp = d_0 + d_1*E + d_2*E^2 + ... + d_n*E^n, where E is the totalVoltage in mV and correctedTemp is in degrees C.
   // NIST uses different coefficients for different temperature subranges: (-200 to 0C), (0 to 500C) and (500 to 1372C).
   if (totalVoltage < 0) { // Temperature is between -200 and 0C.
	  double d[] = {0.0000000E+00, 2.5173462E+01, -1.1662878E+00, -1.0833638E+00, -8.9773540E-01, -3.7342377E-01, -8.6632643E-02, -1.0450598E-02, -5.1920577E-04, 0.0000000E+00};

	  int dLength = sizeof(d) / sizeof(d[0]);
	  for (i = 0; i < dLength; i++) {
		 correctedTemp += d[i] * pow(totalVoltage, i);
	  }
   }
   else if (totalVoltage < 20.644) { // Temperature is between 0C and 500C.
	  double d[] = {0.000000E+00, 2.508355E+01, 7.860106E-02, -2.503131E-01, 8.315270E-02, -1.228034E-02, 9.804036E-04, -4.413030E-05, 1.057734E-06, -1.052755E-08};
	  int dLength = sizeof(d) / sizeof(d[0]);
	  for (i = 0; i < dLength; i++) {
		 correctedTemp += d[i] * pow(totalVoltage, i);
	  }
   }
   else if (totalVoltage < 54.886 ) { // Temperature is between 500C and 1372C.
	  double d[] = {-1.318058E+02, 4.830222E+01, -1.646031E+00, 5.464731E-02, -9.650715E-04, 8.802193E-06, -3.110810E-08, 0.000000E+00, 0.000000E+00, 0.000000E+00};
	  int dLength = sizeof(d) / sizeof(d[0]);
	  for (i = 0; i < dLength; i++) {
		 correctedTemp += d[i] * pow(totalVoltage, i);
	  }
   } else { // NIST only has data for K-type thermocouples from -200C to +1372C. If the temperature is not in that range, set temp to impossible value.
	  correctedTemp = NAN;	  
   }
   return correctedTemp;
}
//------------------------------------------------------------------------------------------------------------------------



