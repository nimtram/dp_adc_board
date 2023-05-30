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

void sd_card_init (void);
void sd_card_deinit (void);

void sd_card_create_file(char* name);
void sd_card_open_file(char* name);
void sd_card_write_to_opened_file(char* name, char * buffer);
void sd_card_close_file(char* name);
#endif /* FILE_HANDLING_H_ */
