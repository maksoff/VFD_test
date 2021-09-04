/*
 * vfd.c
 *
 *  Created on: Jul 14, 2021
 *      Author: makso
 */

#include "vfd.h"




const uint16_t vfd_digits [] = {28686,
								10248,
								24966,
								24970,
								12680,
								20874,
								20878,
								24584,
								29070,
								29066};

const uint16_t vfd_alpha [] =  {29068,
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

const uint16_t vfd_alpha_ru [] = {29068,
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

const uint16_t vfd_special [] = {19970,
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

const char vfd_special_char [] = {'!',
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


const uint8_t vfd_symbols [][2] = {  {1, 16}, // 00 // right <
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

const uint8_t DIGITS = (sizeof(vfd_digits)/sizeof(vfd_digits[0]));
const uint8_t ALPHAS = (sizeof(vfd_alpha)/sizeof(vfd_alpha[0]));
const uint8_t ALPHAR = (sizeof(vfd_alpha_ru)/sizeof(vfd_alpha_ru[0]));
const uint8_t SPECIAL = (sizeof(vfd_special)/sizeof(vfd_special[0]));

uint16_t get_char(char input)
{
	if ('0' <= input && input <= '9')
		return vfd_digits[input - '0'];
	if (0 <= input && input <= 9)
		return vfd_digits[(uint8_t)input];
	for (int i = 0; i < sizeof(vfd_special_char)/sizeof(vfd_special_char[0]); i++)
		if (vfd_special_char[i] == input)
			return vfd_special[i];
	if ('a' <= input && input <= 'z')
		return vfd_alpha[input - 'a'];
	if ('A' <= input && input <= 'Z')
		return vfd_alpha[input - 'A'];
	if (192 <= input && input <= 223)
		return vfd_alpha_ru[input - 192];
	if (224 <= input && input <= 255)
		return vfd_alpha_ru[input - 224];
	return 0;
}

void str2vfd(char * str)
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
		 buf = get_char(*(str++));
		 vfd.arr2[i][0] |= buf & (~(1<<0));
		 vfd.arr2[i][1] |= (buf>>8)&(~(1<<7));
		 if (!--i)
			 break;
	}
}

void symbols_vfd(uint32_t symbols)
{
	for (int i = 0; i < sizeof(vfd_symbols)/sizeof(vfd_symbols[0]); i++)
	{
		if (symbols & (1<<i))
		{
			// set symbol
			for (int b = 0; b < 3; b++)
			  vfd.arr2[vfd_symbols[i][0]][b] |= ((1<<vfd_symbols[i][1])>>(b<<3))&0xFF;
		}
		else
		{
			// reset symbol
			for (int b = 0; b < 3; b++)
			  vfd.arr2[vfd_symbols[i][0]][b] &= ~(((1<<vfd_symbols[i][1])>>(b<<3))&0xFF);
		}
	}
}

void clr_vfd(void)
{
	for (int a = 0; a < sizeof(vfd.arr1); a++)
		  vfd.arr1[a] = 0;
}

uint8_t backup[11*3];

void save_vfd(void)
{
	for (int i = 0; i < sizeof(backup)/sizeof(backup[0]); i++)
		backup[i] = vfd.arr1[i];
}

void restore_vfd(void)
{
	for (int i = 0; i < sizeof(backup)/sizeof(backup[0]); i++)
		vfd.arr1[i] = backup[i];
}
