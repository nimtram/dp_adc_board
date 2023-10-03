

#include "spi_adc.h"
#include "main.h"
#include "stm32h7xx_hal.h"

#define SPI1_CS_PIN GPIOG
#define SPI1_CS_PIN_NUMBER GPIO_PIN_10

#define SPI1_RDY_PIN GPIOA
#define SPI1_RDY_PIN_NUMBER GPIO_PIN_6

#define SPI2_CS_PIN GPIOB
#define SPI2_CS_PIN_NUMBER GPIO_PIN_12

#define SPI2_RDY_PIN GPIOB
#define SPI2_RDY_PIN_NUMBER GPIO_PIN_14

#define SPI4_CS_PIN GPIOE
#define SPI4_CS_PIN_NUMBER GPIO_PIN_4

#define SPI4_RDY_PIN GPIOE
#define SPI4_RDY_PIN_NUMBER GPIO_PIN_5

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;


uint32_t readIndex=0;
uint8_t adc_values[80];

void spi1_adc_init(uint8_t spsValue){
  //GPIOG->PUPDR |= (uint32_t)0x400000;

  uint8_t setupConfigurationRegister[] = {0x20};
//  uint8_t setupConfiguration[] = {0x1F, 0x30};//{0x13, 0x00}; FIXME testing..
  uint8_t setupConfiguration[] = {0x10, 0x00};//{0x13, 0x00}; FIXME testing..

  uint8_t adcModeRegister[] = {0x01};
  uint8_t adcMode[] = {0x0, 0x0};

  uint8_t adcChannelRegister[] = {0x10};
  uint8_t adcChannel[] = {0x80, 0x20};

  uint8_t continuousConvEnableRegister[] = {0x02};
  uint8_t continuousConvEnable[] = {0x00, 0x82};

  uint8_t dataWriteSPS_REGISTER[] = {0x28};
  uint8_t dataWriteSPS[] = {0x05, spsValue}; // 5000SPS = 0x08, 1000SPS = 0x0A, 100SPS = 0x0E, 5SPS = 0x14

  uint8_t dataWriteSyncError_REGISTER[] = {0x06};
  uint8_t dataWriteSyncError[] = {0x00, 0x00};

  /*Start init ADC1*/
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);

  /* Switch AIN0 and AIN1 as inputs due to change in scheme */
  HAL_SPI_Transmit(&hspi1, adcChannelRegister, 1, 100);
  HAL_SPI_Transmit(&hspi1, adcChannel, 2, 100);

  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);

  /* Set external reference to be used */
  HAL_SPI_Transmit(&hspi1, setupConfigurationRegister, 1, 100);
  HAL_SPI_Transmit(&hspi1, setupConfiguration, 2, 100);

  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);

  /* Set continuous conversion mode */
  HAL_SPI_Transmit(&hspi1, adcModeRegister, 1, 100);
  HAL_SPI_Transmit(&hspi1, adcMode, 2, 100);

  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);

  /* Set SPS */
  HAL_SPI_Transmit(&hspi1, dataWriteSPS_REGISTER, 1, 100);
  HAL_SPI_Transmit(&hspi1, dataWriteSPS, 2, 100);

  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);

  /* Set sync pin  */
  HAL_SPI_Transmit(&hspi1, dataWriteSyncError_REGISTER, 1, 100);
  HAL_SPI_Transmit(&hspi1, dataWriteSyncError, 2, 100);

  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);

  /* Set 32bit values and continuous coversion mode */
  HAL_SPI_Transmit(&hspi1, continuousConvEnableRegister, 1, 100);
  HAL_SPI_Transmit(&hspi1, continuousConvEnable, 2, 100);

  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
}

void spi2_adc_init(uint8_t spsValue){
//  GPIOC->PUPDR |= (uint32_t)0x400000;

  uint8_t setupConfigurationRegister[] = {0x20};
//  uint8_t setupConfiguration[] = {0x1F, 0x30};//{0x13, 0x00}; FIXME testing..
  uint8_t setupConfiguration[] = {0x10, 0x00};//{0x13, 0x00}; FIXME testing..

  uint8_t adcModeRegister[] = {0x01};
  uint8_t adcMode[] = {0x0, 0x0};

  uint8_t adcChannelRegister[] = {0x10};
  uint8_t adcChannel[] = {0x80, 0x20};

  uint8_t continuousConvEnableRegister[] = {0x02};
  uint8_t continuousConvEnable[] = {0x00, 0x82};

  uint8_t dataWriteSPS_REGISTER[] = {0x28};
  uint8_t dataWriteSPS[] = {0x05, spsValue}; // 5000SPS = 0x08, 1000SPS = 0x0A, 100SPS = 0x0E, 5SPS = 0x14

  uint8_t dataWriteSyncError_REGISTER[] = {0x06};
  uint8_t dataWriteSyncError[] = {0x00, 0x00};

  /*Start init ADC2*/
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);

  /* Switch AIN0 and AIN1 as inputs due to change in scheme */
  HAL_SPI_Transmit(&hspi2, adcChannelRegister, 1, 100);
  HAL_SPI_Transmit(&hspi2, adcChannel, 2, 100);

  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);

  /* Set external reference to be used */
  HAL_SPI_Transmit(&hspi2, setupConfigurationRegister, 1, 100);
  HAL_SPI_Transmit(&hspi2, setupConfiguration, 2, 100);

  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);

  /* Set continuous conversion mode */
  HAL_SPI_Transmit(&hspi2, adcModeRegister, 1, 100);
  HAL_SPI_Transmit(&hspi2, adcMode, 2, 100);

  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);

  /* Set SPS */
  HAL_SPI_Transmit(&hspi2, dataWriteSPS_REGISTER, 1, 100);
  HAL_SPI_Transmit(&hspi2, dataWriteSPS, 2, 100);

  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);

  /* Set sync pin  */
  HAL_SPI_Transmit(&hspi2, dataWriteSyncError_REGISTER, 1, 100);
  HAL_SPI_Transmit(&hspi2, dataWriteSyncError, 2, 100);

  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);

  /* Set 32bit values and continuous coversion mode */
  HAL_SPI_Transmit(&hspi2, continuousConvEnableRegister, 1, 100);
  HAL_SPI_Transmit(&hspi2, continuousConvEnable, 2, 100);

  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
}

void spi4_adc_init(uint8_t spsValue){
//  GPIOC->PUPDR |= (uint32_t)0x400000;

  uint8_t setupConfigurationRegister[] = {0x20};
//  uint8_t setupConfiguration[] = {0x1F, 0x30};//{0x13, 0x00}; FIXME testing..
  uint8_t setupConfiguration[] = {0x10, 0x00};//{0x13, 0x00}; FIXME testing..

  uint8_t adcModeRegister[] = {0x01};
  uint8_t adcMode[] = {0x0, 0x0};

  uint8_t adcChannelRegister[] = {0x10};
  uint8_t adcChannel[] = {0x80, 0x20};

  uint8_t continuousConvEnableRegister[] = {0x02};
  uint8_t continuousConvEnable[] = {0x00, 0x82};

  uint8_t dataWriteSPS_REGISTER[] = {0x28};
  uint8_t dataWriteSPS[] = {0x05, spsValue}; // 5000SPS = 0x08, 1000SPS = 0x0A, 100SPS = 0x0E, 5SPS = 0x14

  uint8_t dataWriteSyncError_REGISTER[] = {0x06};
  uint8_t dataWriteSyncError[] = {0x00, 0x00};

  /*Start init ADC4*/
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 0);

  /* Switch AIN0 and AIN1 as inputs due to change in scheme */
  HAL_SPI_Transmit(&hspi4, adcChannelRegister, 1, 100);
  HAL_SPI_Transmit(&hspi4, adcChannel, 2, 100);

  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 0);

  /* Set external reference to be used */
  HAL_SPI_Transmit(&hspi4, setupConfigurationRegister, 1, 100);
  HAL_SPI_Transmit(&hspi4, setupConfiguration, 2, 100);

  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 0);

  /* Set continuous conversion mode */
  HAL_SPI_Transmit(&hspi4, adcModeRegister, 1, 100);
  HAL_SPI_Transmit(&hspi4, adcMode, 2, 100);

  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 0);

  /* Set SPS */
  HAL_SPI_Transmit(&hspi4, dataWriteSPS_REGISTER, 1, 100);
  HAL_SPI_Transmit(&hspi4, dataWriteSPS, 2, 100);

  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 0);

  /* Set sync pin  */
  HAL_SPI_Transmit(&hspi4, dataWriteSyncError_REGISTER, 1, 100);
  HAL_SPI_Transmit(&hspi4, dataWriteSyncError, 2, 100);

  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 0);

  /* Set 32bit values and continuous coversion mode */
  HAL_SPI_Transmit(&hspi4, continuousConvEnableRegister, 1, 100);
  HAL_SPI_Transmit(&hspi4, continuousConvEnable, 2, 100);

  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 1);
}


void run_all_adc(void){
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 0);
}





void spi1_read_send_default(){
  uint8_t uartBuffer[20];
  uint32_t varRes = 0;
  uint8_t dataReadRegister[] = {0x44};
  uint8_t dataReadValuesRegister[] = {0xff, 0xff, 0xff, 0xff};//{0x44};
  uint8_t dataReadValues[4] = {};

  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);
  while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==1){} // ready pin
  HAL_SPI_Transmit(&hspi1, dataReadRegister, 1, 100);
  HAL_SPI_TransmitReceive(&hspi1, dataReadValuesRegister, dataReadValues, 4, 100);
  varRes = 0;
  varRes = dataReadValues[0];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[1];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[2];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[3];
  __NOP();

  int tmp = sprintf((char *)uartBuffer, "%lu", varRes);
  HAL_UART_Transmit(&huart5, uartBuffer, (uint16_t) tmp, 10);
  HAL_UART_Transmit(&huart5, (uint8_t *)' ', 1U, 10);
  //HAL_UART_Transmit(&huart5, "\n\r", 2, 10);
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);

}

void spi2_read_send_default(){
  uint8_t uartBuffer[20];
  uint32_t varRes = 0;
  uint8_t dataReadRegister[] = {0x44};
  uint8_t dataReadValuesRegister[] = {0xff, 0xff, 0xff, 0xff};//{0x44};
  uint8_t dataReadValues[4] = {};

  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);
  while(HAL_GPIO_ReadPin(SPI2_RDY_PIN, SPI2_RDY_PIN_NUMBER)==1){} // ready pin
  HAL_SPI_Transmit(&hspi2, dataReadRegister, 1, 100);
  HAL_SPI_TransmitReceive(&hspi2, dataReadValuesRegister, dataReadValues, 4, 100);
  varRes = 0;
  varRes = dataReadValues[0];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[1];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[2];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[3];
  __NOP();

  int tmp = sprintf((char *)uartBuffer, "%lu", varRes);
  HAL_UART_Transmit(&huart5, uartBuffer, (uint16_t) tmp, 10);
  HAL_UART_Transmit(&huart5, (uint8_t *)' ', 1U, 10);
  //HAL_UART_Transmit(&huart5, "\n\r", 2, 10);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 2);
}

void spi1_read_write_continuous_conversion_mode(){
  uint8_t dataReadValues[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t pTxData[] = {0x00, 0x00, 0x00, 0x00};
  uint32_t varRes = 0;
  uint8_t uartBuffer[20];

  //HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);
  while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==1){}
  HAL_SPI_TransmitReceive(&hspi1, pTxData, dataReadValues, 4, 100);
  //HAL_SPI_Receive(&hspi1, dataReadValues, 4, 100);

  varRes = 0;
  varRes = dataReadValues[0];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[1];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[2];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[3];
  __NOP();
  int tmp = sprintf((char *)uartBuffer, "%lu", varRes);
  HAL_UART_Transmit(&huart5, uartBuffer, (uint16_t)tmp, 10);
  HAL_UART_Transmit(&huart5, (uint8_t*)"\r\n", 2, 10);
 // while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==0){}
  //HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
  //HAL_Delay(5);
}



void spi1_read_2k_samples(void){
  uint8_t dataReadValues[4] = {};
  uint8_t pTxData[] = {0x00, 0x00, 0x00, 0x00};

  for (readIndex = 0; readIndex < 80; readIndex = readIndex + 4) {
    while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==1){}
    //HAL_SPI_Receive(&hspi1, dataReadValues, 4, 100);
    HAL_SPI_TransmitReceive(&hspi1, pTxData, dataReadValues, 4, 100);
    adc_values[readIndex]=dataReadValues[0];
    adc_values[readIndex+1]=dataReadValues[1];
    adc_values[readIndex+2]=dataReadValues[2];
    adc_values[readIndex+3]=dataReadValues[3];
    HAL_Delay(10);
    //while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==0){}
  }

}

void spi1_compute_and_send_values(void){
  uint32_t varRes = 0;
  uint8_t uartBuffer[20];
  uint16_t j=0;

  for (j = 0; j < 80; j+=4) {
      varRes = 0;
      varRes = (uint32_t)adc_values[j];
      varRes = varRes<<8;
      varRes = varRes | adc_values[j+1];
      varRes = varRes<<8;
      varRes = varRes | adc_values[j+2];
      varRes = varRes<<8;
      varRes = varRes | adc_values[j+3];

      int tmp = sprintf((char *)uartBuffer, "%lu", varRes);
      HAL_UART_Transmit(&huart5, uartBuffer, (uint16_t)tmp, 10);
      HAL_UART_Transmit(&huart5, (uint8_t*)", ", 2, 10);
      HAL_Delay(10);
  }
}

void spi1_soft_reset(void){
  uint8_t softResetValue[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 0);
  HAL_SPI_Transmit(&hspi1, softResetValue, 8, 100);
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
}

void spi2_soft_reset(void){
  uint8_t softResetValue[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 0);
  HAL_SPI_Transmit(&hspi2, softResetValue, 8, 100);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
}

void spi4_soft_reset(void){
  uint8_t softResetValue[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 0);
  HAL_SPI_Transmit(&hspi4, softResetValue, 8, 100);
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 1);
}


void spi1_read_register(){
  uint8_t uartBuffer[20];
  uint16_t registerValue;
  uint8_t dataReadRegister[] = {0x01};
  uint8_t dataReadValues[2];

  HAL_SPI_Transmit(&hspi2, dataReadRegister, 1, 100);
  HAL_SPI_Receive(&hspi1, dataReadValues, 2, 100);
  __NOP();
  registerValue = (uint16_t)dataReadValues[0];
  registerValue = registerValue << 8;
  registerValue = registerValue | dataReadValues[1];

  int tmp = sprintf((char *)uartBuffer, "%u", registerValue);
  HAL_UART_Transmit(&huart5, (uint8_t*)"Hodnota:", 8U, 10);
  HAL_UART_Transmit(&huart5, uartBuffer, tmp, 10);
  HAL_UART_Transmit(&huart5, (uint8_t*)"\r\n", 2, 10);
}

void spi1_read_it_mode(){
  uint8_t dataReadValues[4] = {};
  uint8_t pTxData[] = {0x00, 0x00, 0x00, 0x00};
  uint32_t varRes = 0;
  uint8_t uartBuffer[20];

  HAL_SPI_TransmitReceive_DMA(&hspi1, pTxData, dataReadValues, 4);
  //HAL_SPI_TransmitReceive(&hspi1, pTxData, dataReadValues, 4, 100);
  //HAL_SPI_Receive(&hspi1, dataReadValues, 4, 100);
  HAL_Delay(100);
  varRes = 0;
  varRes = dataReadValues[0];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[1];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[2];
  varRes = varRes<<8;
  varRes = varRes | dataReadValues[3];
  __NOP();
  int tmp = sprintf((char *)uartBuffer, "%lu", varRes);
  HAL_UART_Transmit(&huart5, uartBuffer, (uint16_t)tmp, 10);
  HAL_UART_Transmit(&huart5, (uint8_t*)"\r\n", 2, 10);
 // while(HAL_GPIO_ReadPin(SPI1_RDY_PIN, SPI1_RDY_PIN_NUMBER)==0){}
}

void spi_it_convert_and_send(uint8_t* adcRawVaues, uint32_t valuesToSend){
  uint32_t varRes = 0;
  uint8_t uartBuffer[20];
  uint16_t j=0;

  for (j = 0; j < (valuesToSend*4); j+=4) {
      varRes = 0;
      varRes = (uint32_t)adcRawVaues[j];
      varRes = varRes<<8;
      varRes = varRes | adcRawVaues[j+1];
      varRes = varRes<<8;
      varRes = varRes | adcRawVaues[j+2];
      varRes = varRes<<8;
      varRes = varRes | adcRawVaues[j+3];

      int tmp = sprintf((char *)uartBuffer, "%12lu", varRes);
      HAL_UART_Transmit(&huart4, uartBuffer, (uint16_t)tmp, 10);
      if(j <((valuesToSend-1)*4)){
        HAL_UART_Transmit(&huart4, (uint8_t*)",", 1, 10);
      }
      //HAL_Delay(10);
  }
  HAL_UART_Transmit(&huart4, (uint8_t*)"\n\r", 2, 10);
}


void spi_send_all_three_values(uint8_t* adcRawValue_x, uint8_t* adcRawValue_y, uint8_t* adcRawValue_z){
  uint32_t value_x = 0;
  uint32_t value_y = 0;
  uint32_t value_z = 0;
  uint8_t uartBuffer_x[20];
  uint8_t uartBuffer_y[20];
  uint8_t uartBuffer_z[20];

  value_x = (uint32_t)adcRawValue_x[0];
  value_x = value_x<<8;
  value_x = value_x | adcRawValue_x[1];
  value_x = value_x<<8;
  value_x = value_x | adcRawValue_x[2];
  value_x = value_x<<8;
  value_x = value_x | adcRawValue_x[3];

  value_y = (uint32_t)adcRawValue_y[0];
  value_y = value_y<<8;
  value_y = value_y | adcRawValue_y[1];
  value_y = value_y<<8;
  value_y = value_y | adcRawValue_y[2];
  value_y = value_y<<8;
  value_y = value_y | adcRawValue_y[3];

  value_z = (uint32_t)adcRawValue_z[0];
  value_z = value_z<<8;
  value_z = value_z | adcRawValue_z[1];
  value_z = value_z<<8;
  value_z = value_z | adcRawValue_z[2];
  value_z = value_z<<8;
  value_z = value_z | adcRawValue_z[3];

  int length_x = sprintf((char *)uartBuffer_x, "%12lu", value_x);
  int length_y = sprintf((char *)uartBuffer_y, "%12lu", value_y);
  int length_z = sprintf((char *)uartBuffer_z, "%12lu", value_z);

  HAL_UART_Transmit(&huart4, "x: ", 3, 10);
  HAL_UART_Transmit(&huart4, uartBuffer_x, (uint16_t)length_x, 10);
  HAL_UART_Transmit(&huart4, ", y: ", 5, 10);
  HAL_UART_Transmit(&huart4, uartBuffer_y, (uint16_t)length_y, 10);
  HAL_UART_Transmit(&huart4, ", z: ", 5, 10);
  HAL_UART_Transmit(&huart4, uartBuffer_z, (uint16_t)length_z, 10);
  HAL_UART_Transmit(&huart4, "\n", 1, 10);

}

void spi1_set_exti(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP | GPIO_MODE_IT_FALLING; //GPIO_MODE_AF_PP  | TRIGGER_RISING | TRIGGER_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void spi2_set_exti(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;// | EXTI_IT | TRIGGER_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}



void setNewSPStoAllADCs(uint8_t spsValue){
  HAL_GPIO_WritePin(SPI1_CS_PIN, SPI1_CS_PIN_NUMBER, 1);
  HAL_GPIO_WritePin(SPI2_CS_PIN, SPI2_CS_PIN_NUMBER, 1);
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 1);
  spi1_soft_reset();
  spi2_soft_reset();
  spi4_soft_reset();
  spi1_adc_init(spsValue);
  spi2_adc_init(spsValue);
  spi4_adc_init(spsValue);
}

void readRegister(void){
  volatile uint8_t pRxData[] = {0x00, 0x00, 0x00};
  volatile uint8_t pTxData[] = {0x60, 0x00, 0x00};
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 0);
  HAL_SPI_TransmitReceive(&hspi4, pTxData, pRxData,3,100);
  HAL_GPIO_WritePin(SPI4_CS_PIN, SPI4_CS_PIN_NUMBER, 1);
  __NOP();
}
