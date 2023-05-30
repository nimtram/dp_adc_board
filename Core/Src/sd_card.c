
#include "sd_card.h"

FRESULT res; /* FatFs function common result code */
uint32_t byteswritten, bytesread; /* File write/read counts */
uint8_t wtext[] = "STM32 FATFS works great!"; /* File write buffer */
uint8_t rtext[_MAX_SS];/* File read buffer */
FILINFO fno;

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

  //res = f_stat (name, &fno);
  res = f_open(&SDFile, name, FA_CREATE_ALWAYS | FA_WRITE);
  if (res != FR_OK){
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
