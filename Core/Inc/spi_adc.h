/*
 * file_handling.h
 *
 *  Created on: 26-Jul-2019
 *      Author: arunr
 */

#ifndef SPI_ADC_H_
#define SPI_ADC_H_


#include "string.h"
#include "stdio.h"

void spi1_soft_reset(void);
void spi2_soft_reset(void);
void spi4_soft_reset(void);
void spi1_set_exti(void);
void spi1_read_2k_samples(void);
void spi1_compute_and_send_values(void);
void spi1_read_write_continuous_conversion_mode(void);
void spi1_read_it_mode();

// used now:
void spi1_adc_init(uint8_t spsValue);
void spi2_adc_init(uint8_t spsValue);
void spi4_adc_init(uint8_t spsValue);
void run_all_adc(void);
void setNewSPStoAllADCs(uint8_t spsValue);
void spi_it_convert_and_send(uint8_t* adcRawVaues, uint32_t valuesToSend);
void getStringFromValues(uint32_t adcRawValue_x, uint32_t adcRawValue_y, uint32_t adcRawValue_z, char* concatenatedBuffer);
void getStringFromValuesFloat(uint32_t adcRawValue_x, uint32_t adcRawValue_y, uint32_t adcRawValue_z, char* concatenatedBuffer, char resolutionX, char resolutionY, char resolutionZ);
void readRegister(void);
void formatFloatString(char* str, float floatValue);
#endif /* FILE_HANDLING_H_ */
