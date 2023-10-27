/*
 * file_handling.h
 *
 *  Created on: 26-Jul-2019
 *      Author: arunr
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

bool sd_card_init (void);
void sd_card_deinit (void);
bool sd_card_open_file(void);
void sd_card_close_file(void);
bool sd_card_write_to_opened_file(char * buffer);


void sd_card_create_file(char* name);



void sd_card_test(void);
void sd_card_write_values(char* name, char* buffer, uint32_t amountOfValues);
#endif /* FILE_HANDLING_H_ */
