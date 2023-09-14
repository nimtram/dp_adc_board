
#include "sd_card.h"
#include <stdbool.h>
FRESULT res; /* FatFs function common result code */
uint32_t byteswritten, bytesread; /* File write/read counts */
uint8_t wtext[] = "STM32 FATFS works great!"; /* File write buffer */
uint8_t rtext[_MAX_SS];/* File read buffer */
FILINFO fno;
uint8_t nameFileNumber = 1;
bool newMeasurement = false;

void sd_card_init (void)
{
  res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
  if (res != FR_OK){
    __NOP(); // TODO error handling
  }
}

void sd_card_deinit (void)
{
  res = f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
  if (res != FR_OK){
    __NOP(); // TODO error handling
  }
}


void sd_card_create_file(char* name){
  FRESULT localRes = FR_OK;
  //res = f_stat (name, &fno);
  if (newMeasurement == true){

    while (localRes == FR_OK){
      localRes = f_open(&SDFile, name, FA_WRITE);
      if (localRes == FR_OK){
        nameFileNumber = nameFileNumber +1;
        name[3] = nameFileNumber + '0';
        f_close(&SDFile);
      }

    }
    if (localRes == FR_NO_FILE){
      localRes = f_open(&SDFile, name, FA_CREATE_ALWAYS | FA_WRITE);
    }
    newMeasurement = false;
  }else{
      localRes = f_open(&SDFile, name, FA_CREATE_ALWAYS | FA_WRITE);
    }

  if (localRes != FR_OK){
    __NOP(); // TODO error handling
  }

  /*res = f_write(&SDFile, wtext, strlen((char *)wtext), (void *)&byteswritten);

   if((byteswritten == 0) || (res != FR_OK))
   {
     __NOP(); // TODO error handling
   }*/

   f_close(&SDFile);

}
void sd_card_open_file(char* name){

  res = f_open(&SDFile, name, FA_WRITE);
  if (res != FR_OK){
    __NOP(); // TODO error handling
  }

}

void sd_card_write_to_opened_file(char* name, char * buffer){


  res = f_write(&SDFile, buffer, strlen((char *)buffer), (void *)&byteswritten);

   if((byteswritten == 0) || (res != FR_OK))
   {
     __NOP(); // TODO error handling
   }

}

void sd_card_close_file(char* name){

  f_close(&SDFile);
}

void sd_card_test(void){
  sd_card_create_file("test2.txt");
  sd_card_open_file("test2.txt");

  char buffer[10];

  for (uint8_t var = 0; var < 200; ++var) {
    sprintf(buffer, "%d", var);
    sd_card_write_to_opened_file("test2.txt", buffer);
  }
  sd_card_close_file("test2.txt");
}

void sd_card_write_values(char* name, char* buffer, uint32_t amountOfValues){
  sd_card_create_file(name);
  sd_card_open_file(name);

  uint32_t varRes = 0;
  char uartBuffer[20];
  uint16_t j=0;

  for (j = 0; j < (amountOfValues*4); j+=4) {
       varRes = 0;
       varRes = (uint32_t)buffer[j];
       varRes = varRes<<8;
       varRes = varRes | buffer[j+1];
       varRes = varRes<<8;
       varRes = varRes | buffer[j+2];
       varRes = varRes<<8;
       varRes = varRes | buffer[j+3];

       int tmp = sprintf((char *)uartBuffer, "%lu", varRes);
       sd_card_write_to_opened_file(name, uartBuffer);
       sd_card_write_to_opened_file(name, (char*) "\n");
   }

  sd_card_close_file(name);
}
