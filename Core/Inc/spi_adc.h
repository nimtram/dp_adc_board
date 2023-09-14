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
void spi1_set_exti(void);
void spi1_read_2k_samples(void);
void spi1_compute_and_send_values(void);
void spi1_read_write_continuous_conversion_mode(void);
void spi1_read_it_mode();

// used now:
void spi1_adc_init(void);
void spi2_adc_init(void);
void spi_it_convert_and_send(uint8_t* adcRawVaues, uint32_t valuesToSend);


#endif /* FILE_HANDLING_H_ */
