/*
 * vfd.h
 *
 *  Created on: Jul 12, 2021
 *      Author: makso
 */

#ifndef INC_VFD_H_
#define INC_VFD_H_

#include "main.h"

typedef enum {
	VFD_CS_LOW = 0,
	VFD_CS_HIGH = 1
} vfd_cs_t;

// these functions should be implemented by user
void vfd_spi_cs(vfd_cs_t cs);
void vfd_spi_tx(uint8_t *pData, uint16_t Size);

void vfd_update(void);
void vfd_leds(uint8_t leds);

uint16_t _vfd_get_char_code(char input);
void vfd_put_string(char * str);
void vfd_clear_buf(void);

void vfd_store_backup(void);
void vfd_restore_backup(void);
void vfd_put_symbols(uint32_t symbols);
void vfd_set_symbols(uint32_t symbols);
void vfd_clr_symbols(uint32_t symbols);

extern const uint16_t _VFD_MAP_DIGITS [];
extern const uint16_t _VFD_MAP_ALPHA [];
extern const uint16_t _VFD_MAP_ALPHA_RU [];
extern const uint16_t _VFD_MAP_SPECIAL [];
extern const char _VFD_MAP_SPECIAL_CHAR [];

extern const uint8_t _VFD_SIZE_DIGITS;
extern const uint8_t _VFD_SIZE_ALPHAS;
extern const uint8_t _VFD_SIZE_ALPHARU;
extern const uint8_t _VFD_SIZE_SPECIAL;

union VFD {
	  uint8_t arr2[11][3];
	  uint8_t arr1[11*3];
} vfd;

enum {
	VFD_SYM_ARROW1  = 1<<0,
	VFD_SYM_ARROW2  = 1<<1,
	VFD_SYM_BAR1	 = 1<<2,
	VFD_SYM_BAR2	 = 1<<3,
	VFD_SYM_BAR3	 = 1<<4,
	VFD_SYM_BAR4	 = 1<<5,
	VFD_SYM_BAR5	 = 1<<6,
	VFD_SYM_BAR6	 = 1<<7,
	VFD_SYM_BAR7	 = 1<<8,
	VFD_SYM_BAR8	 = 1<<9,
	VFD_SYM_BAR9	 = 1<<10,
	VFD_SYM_BAR10	 = 1<<11,
	VFD_SYM_C		 = 1<<12,
	VFD_SYM_B		 = 1<<13,
	VFD_SYM_DOLBY	 = 1<<14,
	VFD_SYM_ARROW_RIGHT	 = 1<<15,
	VFD_SYM_ARROW_LEFT	 = 1<<16,
	VFD_SYM_DIGITAL 	 = 1<<17,
	VFD_SYM_ANALOG		 = 1<<18,
	VFD_SYM_BRACKET_RIGHT	 = 1<<19,
	VFD_SYM_BRACKET_LEFT	 = 1<<20,
	VFD_SYM_SMALL_ARROW_LEFT	 = 1<<21,
	VFD_SYM_SMALL_ARROW_RIGHT	 = 1<<22,
	VFD_SYM_DCC		 = 1<<23,
	VFD_SYM_COLON	 = 1<<24,
};

#endif /* INC_VFD_H_ */
