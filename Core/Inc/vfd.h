/*
 * vfd.h
 *
 *  Created on: Jul 12, 2021
 *      Author: makso
 */

#ifndef INC_VFD_H_
#define INC_VFD_H_

#include "main.h"

uint16_t get_char(char input);
void str2vfd(char * str);
void clr_vfd(void);

void save_vfd(void);
void restore_vfd(void);
void symbols_vfd(uint32_t symbols);

extern const uint16_t vfd_digits [];
extern const uint16_t vfd_alpha [];
extern const uint16_t vfd_alpha_ru [];
extern const uint16_t vfd_special [];
extern const char vfd_special_char [];

extern const uint8_t DIGITS;
extern const uint8_t ALPHAS;
extern const uint8_t ALPHAR;
extern const uint8_t SPECIAL;

union VFD {
	  uint8_t arr2[11][3];
	  uint8_t arr1[11*3];
} vfd;

char txt2disp[128]; // storage for string received over CDC
bool fresh_txt;


#endif /* INC_VFD_H_ */
