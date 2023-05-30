//
//
//#include "spi_adc.h"
//
//
//void spi1_adc_init(void){
//  GPIOG->PUPDR |= (uint32_t)0x400000;
//  uint8_t adcModeRegister[] = {0x01};
//  uint8_t adcMode[] = {0x0, 0x0};
//
//  uint8_t continuousConvEnableRegister[] = {0x02};
//  uint8_t continuousConvEnable[] = {0x00, 0x82};
//
//  uint8_t dataWriteSPS[] = {0x05, 0x14}; // 5000SPS = 0x08, 1000SPS = 0x0A
//  uint8_t dataWriteSPS_REGISTER[] = {0x28};
//
//  uint8_t dataWrite32BitValues_REGISTER[] = {0x02};
//  uint8_t dataWrite32BitValues[] = {0x00, 0x02};
//
//  // GPIO
//  uint8_t dataWriteSyncError[] = {0x00, 0x00};
//  uint8_t dataWriteSyncError_REGISTER[] = {0x06};
//
//
//
//  /*Start init ADC1*/
//  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);
//
////  /* Set continuous conversion mode */
//  HAL_SPI_Transmit(&hspi1, adcModeRegister, 1, 100);
//  HAL_SPI_Transmit(&hspi1, adcMode, 2, 100);
//
//  /* Set SPS */
//  HAL_SPI_Transmit(&hspi1, dataWriteSPS_REGISTER, 1, 100);
//  HAL_SPI_Transmit(&hspi1, dataWriteSPS, 2, 100);
//
//  //  /* Set sync pin  */
//  HAL_SPI_Transmit(&hspi1, dataWriteSyncError_REGISTER, 1, 100);
//  HAL_SPI_Transmit(&hspi1, dataWriteSyncError, 2, 100);
//
//  /* Set 32bit values and continuous noversion mode */
//  HAL_SPI_Transmit(&hspi1, continuousConvEnableRegister, 1, 100);
//  HAL_SPI_Transmit(&hspi1, continuousConvEnable, 2, 100);
//
//  /* CS -> Start */
//  /*for (int tmpCnt = 0; tmpCnt < 1000; ++tmpCnt) {
//    HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
//  }*/
//
//  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
//  HAL_Delay(100);
//  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);
//}
//
//void spi2_adc_init(void){
//  uint8_t dataWriteSPS[] = {0x05, 0x0A}; // 5000SPS = 0x08, 1000SPS = 0x0A
//  uint8_t dataWriteSPS_REGISTER[] = {0x28};
//
//  uint8_t dataWrite32BitValues[] = {0x00, 0x02};
//  uint8_t dataWrite32BitValues_REGISTER[] = {0x02};
//
//  /*Start init ADC1*/
//  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);
//  /* Set 32bit values */
//  HAL_SPI_Transmit(&hspi2, dataWrite32BitValues_REGISTER, 1, 100);
//  HAL_SPI_Transmit(&hspi2, dataWrite32BitValues, 2, 100);
//
//
//  /* Set SPS */
//  HAL_SPI_Transmit(&hspi2, dataWriteSPS_REGISTER, 1, 100);
//  HAL_SPI_Transmit(&hspi2, dataWriteSPS, 2, 100);
//
//  /* CS -> Start */
//  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
//  HAL_Delay(100);
//  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);
//}
//
//
//void spi1_read_send_default(){
//  char uartBuffer[20];
//  uint32_t varRes = 0;
//  uint8_t dataReadRegister[] = {0x44};
//  uint8_t dataReadValuesRegister[] = {0xff, 0xff, 0xff, 0xff};//{0x44};
//  uint8_t dataReadValues[4] = {};
//
//  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);
//  while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==1){} // ready pin
//  HAL_SPI_Transmit(&hspi1, dataReadRegister, 1, 100);
//  HAL_SPI_TransmitReceive(&hspi1, dataReadValuesRegister, dataReadValues, 4, 100);
//  varRes = 0;
//  varRes = dataReadValues[0];
//  varRes = varRes<<8;
//  varRes = varRes | dataReadValues[1];
//  varRes = varRes<<8;
//  varRes = varRes | dataReadValues[2];
//  varRes = varRes<<8;
//  varRes = varRes | dataReadValues[3];
//  __NOP();
//
//  int tmp = sprintf(uartBuffer, "%lu", varRes);
//  HAL_UART_Transmit(&huart5, uartBuffer, tmp, 10);
//  HAL_UART_Transmit(&huart5, " ", 1, 10);
//  //HAL_UART_Transmit(&huart5, "\n\r", 2, 10);
//  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
//
//}
//
//void spi2_read_send_default(){
//  uint8_t uartBuffer[20];
//  uint32_t varRes = 0;
//  uint8_t dataReadRegister[] = {0x44};
//  uint8_t dataReadValuesRegister[] = {0xff, 0xff, 0xff, 0xff};//{0x44};
//  uint8_t dataReadValues[4] = {};
//
//  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);
//  while(HAL_GPIO_ReadPin(SPI2_RDY_PIN, SPI2_RDY_PIN_NUMBER)==1){} // ready pin
//  HAL_SPI_Transmit(&hspi2, dataReadRegister, 1, 100);
//  HAL_SPI_TransmitReceive(&hspi2, dataReadValuesRegister, dataReadValues, 4, 100);
//  varRes = 0;
//  varRes = dataReadValues[0];
//  varRes = varRes<<8;
//  varRes = varRes | dataReadValues[1];
//  varRes = varRes<<8;
//  varRes = varRes | dataReadValues[2];
//  varRes = varRes<<8;
//  varRes = varRes | dataReadValues[3];
//  __NOP();
//
//  int tmp = sprintf(uartBuffer, "%u", varRes);
//  HAL_UART_Transmit(&huart5, uartBuffer, tmp, 10);
//  HAL_UART_Transmit(&huart5, " ", 1, 10);
//  //HAL_UART_Transmit(&huart5, "\n\r", 2, 10);
//  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
//  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
//  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
//  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 2);
//}
//
//void spi1_read_write_continuous_conversion_mode(){
//  uint8_t dataReadValues[4] = {};
//  uint8_t pTxData[] = {0x00, 0x00, 0x00, 0x00};
//  uint32_t varRes = 0;
//  uint8_t uartBuffer[20];
//
//  while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==1){}
//  HAL_SPI_TransmitReceive(&hspi1, pTxData, dataReadValues, 4, 100);
//  //HAL_SPI_Receive(&hspi1, dataReadValues, 4, 100);
//
//  varRes = 0;
//  varRes = dataReadValues[0];
//  varRes = varRes<<8;
//  varRes = varRes | dataReadValues[1];
//  varRes = varRes<<8;
//  varRes = varRes | dataReadValues[2];
//  varRes = varRes<<8;
//  varRes = varRes | dataReadValues[3];
//  __NOP();
//  int tmp = sprintf(uartBuffer, "%u", varRes);
//  HAL_UART_Transmit(&huart5, uartBuffer, tmp, 10);
//  HAL_UART_Transmit(&huart5, "\r\n", 2, 10);
// // while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==0){}
//}
//
//
//
//void spi1_read_2k_samples(void){
//  uint8_t dataReadValues[4] = {};
//  uint8_t pTxData[] = {0x00, 0x00, 0x00, 0x00};
//
//  for (readIndex = 0; readIndex < 80; readIndex = readIndex + 4) {
//    while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==1){}
//    //HAL_SPI_Receive(&hspi1, dataReadValues, 4, 100);
//    HAL_SPI_TransmitReceive(&hspi1, pTxData, dataReadValues, 4, 100);
//    adc_values[readIndex]=dataReadValues[0];
//    adc_values[readIndex+1]=dataReadValues[1];
//    adc_values[readIndex+2]=dataReadValues[2];
//    adc_values[readIndex+3]=dataReadValues[3];
//    HAL_Delay(10);
//    //while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==0){}
//  }
//
//}
//
//void spi1_compute_and_send_values(void){
//  uint32_t varRes = 0;
//  uint8_t uartBuffer[20];
//  uint16_t j=0;
//
//  for (j = 0; j < 80; j+=4) {
//      varRes = 0;
//      varRes = (uint32_t)adc_values[j];
//      varRes = varRes<<8;
//      varRes = varRes | adc_values[j+1];
//      varRes = varRes<<8;
//      varRes = varRes | adc_values[j+2];
//      varRes = varRes<<8;
//      varRes = varRes | adc_values[j+3];
//
//      int tmp = sprintf(uartBuffer, "%lu", varRes);
//      HAL_UART_Transmit(&huart5, uartBuffer, tmp, 10);
//      HAL_UART_Transmit(&huart5, ", ", 2, 10);
//      HAL_Delay(10);
//  }
//}
//
//void spi1_soft_reset(void){
//  uint8_t softResetValue[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
//  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);
//  HAL_SPI_Transmit(&hspi1, softResetValue, 8, 100);
//  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
//}
//
//
//
//void spi1_read_register(){
//  uint8_t uartBuffer[20];
//  uint16_t registerValue;
//  uint8_t dataReadRegister[] = {0x01};
//  uint8_t dataReadValues[2];
//
//  HAL_SPI_Transmit(&hspi2, dataReadRegister, 1, 100);
//  HAL_SPI_Receive(&hspi1, dataReadValues, 2, 100);
//  __NOP();
//  registerValue = (uint16_t)dataReadValues[0];
//  registerValue = registerValue << 8;
//  registerValue = registerValue | dataReadValues[1];
//
//  int tmp = sprintf(uartBuffer, "%u", registerValue);
//  HAL_UART_Transmit(&huart5, "Hodnota:", 8, 10);
//  HAL_UART_Transmit(&huart5, uartBuffer, tmp, 10);
//  HAL_UART_Transmit(&huart5, "\r\n", 2, 10);
//}
