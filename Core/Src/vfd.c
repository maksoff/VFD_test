/*
 * vfd.c
 *
 *  Created on: Jul 14, 2021
 *      Author: makso
 */

#include "vfd.h"


uint8_t _VFD_BUF_BACKUP[11*3];

/**** code for VFD display FUTABA FV651G to output the digits ****/
// digits from 0 to 9
const uint16_t _VFD_MAP_DIGITS [] = {28686,
								10248,
								24966,
								24970,
								12680,
								20874,
								20878,
								24584,
								29070,
								29066};

// alphas from A to Z
const uint16_t _VFD_MAP_ALPHA [] =  {29068,
								26026,
								20486,
								25642,
								20870,
								20868,
								20750,
								12684,
								17442,
								8206,
								6340,
								4102,
								14860,
								12876,
								28686,
								29060,
								28750,
								29124,
								20874,
								17440,
								12302,
								6164,
								12380,
								2640,
								2592,
								18450};

const uint16_t _VFD_MAP_ALPHA_RU [] = {29068,
								17834,
								26026,
								20484,
								25642,
								20870,
								//20870, doesn't accept Ё
								3696,
								18698,
								14364,
								14364,
								6340,
								10264,
								14860,
								12684,
								28686,
								28684,
								29060,
								20486,
								17440,
								12682,
								30112,
								2640,
								12366,
								12680,
								13358,
								13422,
								4244,
								12444,
								4244,
								24842,
								14540,
								29080};


// table for special characters and lower case letters
const uint16_t _VFD_MAP_SPECIAL [] = {19970,
								24864,
								4080,
								1440,
								384,
								2,
								2048,
								9216,
								7128,
								2112,
								528,
								21930,
								2064,
								1056,
								576,
								19034,
								29958,
								29056,
								8590,
								2112,
								528,
								1442,
								4492,
								16,
								32,
								24974,
};

// lookup table for special characters
const char _VFD_MAP_SPECIAL_CHAR [] = {'!',
									'?',
									'*',
									'+',
									'-',
									'_',
									'\'',
									'"',
									'%',
									'(',
									')',
									'$',
									'/',
									'|',
									'\\',
									'&',
									'@',
									176, // °
									'd',
									'<',
									'>',
									177, // ±
									'h',
									',',
									'.',
									'a',
};


// table for additional symbols on this display
const uint8_t _VFD_MAP_SYMBOLS [][2] = {  {1, 16}, // 00 // right <
							   {1, 15}, // right <<
							   {2, 16},
							   {2, 15},
							   {3, 16},
							   {3, 15},
							   {4, 16},
							   {4, 15},
							   {5, 16},
							   {5, 15},
							   {6, 16},
							   {6, 15}, // 11 // most left bars
							   {8, 16},	//  c
							   {8, 15},	// b
							   {9, 16},	// dolby
							   {10, 16}, 	// >
							   {10, 15},	// <
		    				   {0, 0}, // digital
							   {0, 1}, // analog
							   {0, 4}, // )
							   {0, 3}, // (
							   {0, 5}, // <-
							   {0, 2}, // ->
							   {0, 6}, // dcc
							   {6, 0}, // :
};

const uint8_t _VFD_SIZE_DIGITS = (sizeof(_VFD_MAP_DIGITS)/sizeof(_VFD_MAP_DIGITS[0]));
const uint8_t _VFD_SIZE_ALPHAS = (sizeof(_VFD_MAP_ALPHA)/sizeof(_VFD_MAP_ALPHA[0]));
const uint8_t _VFD_SIZE_ALPHARU = (sizeof(_VFD_MAP_ALPHA_RU)/sizeof(_VFD_MAP_ALPHA_RU[0]));
const uint8_t _VFD_SIZE_SPECIAL = (sizeof(_VFD_MAP_SPECIAL)/sizeof(_VFD_MAP_SPECIAL[0]));
const uint8_t _VFD_SIZE_SYMBOLS = (sizeof(_VFD_MAP_SYMBOLS)/sizeof(_VFD_MAP_SYMBOLS[0]));

/**
 * Returns code for display from character code
 */
uint16_t _vfd_get_char_code(char code)
{
	if ('0' <= code && code <= '9')
		return _VFD_MAP_DIGITS[code - '0'];
	if (0 <= code && code <= 9)
		return _VFD_MAP_DIGITS[(uint8_t)code];
	for (int i = 0; i < _VFD_SIZE_SPECIAL; i++)
		if (_VFD_MAP_SPECIAL_CHAR[i] == code)
			return _VFD_MAP_SPECIAL[i];
	if ('a' <= code && code <= 'z')
		return _VFD_MAP_ALPHA[code - 'a'];
	if ('A' <= code && code <= 'Z')
		return _VFD_MAP_ALPHA[code - 'A'];
	if (192 <= code && code <= 223) // russian upper letters
		return _VFD_MAP_ALPHA_RU[code - 192];
	if (224 <= code && code <= 255) // russian lower letters
		return _VFD_MAP_ALPHA_RU[code - 224];
	return 0;
}

/**
 * put string in buffer
 */
void vfd_put_string(char * str)
{
	uint16_t buf;
	// erase letters only
	for (int i = 10; i > 0; i --)
	{
		 vfd.arr2[i][0] &= 1<<0;
		 vfd.arr2[i][1] &= 1<<7;
	}
	uint8_t i = 10;
	while (*str)
	{
		 buf = _vfd_get_char_code(*(str++));
		 vfd.arr2[i][0] |= buf & (~(1<<0));
		 vfd.arr2[i][1] |= (buf>>8)&(~(1<<7));
		 if (!--i)
			 break;
	}
}

/**
 * put symbols mask in buffer
 */
void vfd_put_symbols(uint32_t symbols)
{
	for (int i = 0; i < _VFD_SIZE_SYMBOLS; i++)
	{
		if (symbols & (1<<i))
		{
			// set symbol
			for (int b = 0; b < 3; b++)
			  vfd.arr2[_VFD_MAP_SYMBOLS[i][0]][b] |= ((1<<_VFD_MAP_SYMBOLS[i][1])>>(b<<3))&0xFF;
		}
		else
		{
			// reset symbol
			for (int b = 0; b < 3; b++)
			  vfd.arr2[_VFD_MAP_SYMBOLS[i][0]][b] &= ~(((1<<_VFD_MAP_SYMBOLS[i][1])>>(b<<3))&0xFF);
		}
	}
}

/**
 * set symbols mask in buffer
 */
void vfd_set_symbols(uint32_t symbols)
{
	for (int i = 0; i < _VFD_SIZE_SYMBOLS; i++)
	{
		if (symbols & (1<<i))
		{
			// set symbol
			for (int b = 0; b < 3; b++)
			  vfd.arr2[_VFD_MAP_SYMBOLS[i][0]][b] |= ((1<<_VFD_MAP_SYMBOLS[i][1])>>(b<<3))&0xFF;
		}
	}
}

/**
 * put symbols mask in buffer
 */
void vfd_clr_symbols(uint32_t symbols)
{
	for (int i = 0; i < _VFD_SIZE_SYMBOLS; i++)
	{
		if (symbols & (1<<i))
		{
			// reset symbol
			for (int b = 0; b < 3; b++)
			  vfd.arr2[_VFD_MAP_SYMBOLS[i][0]][b] &= ~(((1<<_VFD_MAP_SYMBOLS[i][1])>>(b<<3))&0xFF);
		}
	}
}

/**
 * clear buffer
 */
void vfd_clear_buf(void)
{
	for (int a = 0; a < sizeof(vfd.arr1); a++)
		  vfd.arr1[a] = 0;
}

/**
 * backup data
 */
void vfd_store_backup(void)
{
	for (int i = 0; i < sizeof(_VFD_BUF_BACKUP)/sizeof(_VFD_BUF_BACKUP[0]); i++)
		_VFD_BUF_BACKUP[i] = vfd.arr1[i];
}

/**
 * restore data
 */
void vfd_restore_backup(void)
{
	for (int i = 0; i < sizeof(_VFD_BUF_BACKUP)/sizeof(_VFD_BUF_BACKUP[0]); i++)
		vfd.arr1[i] = _VFD_BUF_BACKUP[i];
}

/**
 * update data on VFD display
 */
void vfd_update(void) {
	uint8_t data = 0b11000000; // command 3, set address to 0
	vfd_spi_cs(VFD_CS_LOW);
	vfd_spi_tx(&data, 1);
	vfd_spi_tx(vfd.arr1, sizeof(vfd.arr1));
	vfd_spi_cs(VFD_CS_HIGH);
}

/**
 * enable disable leds mask (4 leds available)
 */
void vfd_leds(uint8_t leds)
{
	uint8_t data = 0b01000001; // command 2, write to LED port
	vfd_spi_cs(VFD_CS_LOW);
	vfd_spi_tx(&data, 1);
	data = (~leds)&0b1111;
	vfd_spi_tx(&data, 1);
	vfd_spi_cs(VFD_CS_HIGH);
}
