/*
 * microrl_cmd.c
 *
 *  Created on: Jun 23, 2021
 *      Author: makso
 */

#include "main.h"
#include "microrl_cmd.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

microrl_t mcrl;
microrl_t * p_mcrl = &mcrl;

bool color_out = true;

/********************************
 *
 */

/*
 * Ex. Menu Structure:						should be formated in this way:
 * help 	-- help message					{0, 	"help", "help message", print_help},
 * h		-- -//-							{-1, 	"h", 	"", 			NULL},
 * clr		-- clear screen					{0,		"clr",  "clear screen",	clear_scr},
 * led  	-- led toggle					{0, 	"led",  "led toggle,	led_toggle},
 * lamp  	-- -//-							{-1,	"lamp",	"",				NULL},
 *   on 	-- turn led on					{1, 	"on",	"turn led on",	led_on},
 *   off 	-- turn led off					{1, 	"off",	"turn led off",	led_off},
 * time		-- show time once				{0,		"time",	"show time once", print_time},
 *   show   -- autoupdate time				{1,		"show",	"autoupdate time", print_time_auto},
 *   auto	-- -//-							{-1,	"auto", "",				NULL},
 *     simple -- autoupdate without esc		{2,		"simple", "autoupdate without esc", print_time_no_esc}
 *
 * !      -//- == synonym for function above
 * !!!    order of lines is important! the alternative names and sublevel commands are referenced for function above.
 */

const microrl_action_t microrl_actions [] =
{
		{ 0, 		"help", 	"this message", 			print_help},
		{-1,		"h", 		"", 						NULL},
		{-1,		"?", 		"", 						NULL},
		{ 0,		"color",	"toggle spec characters",	color_toggle},
		{   1,		"on",		"turn on",					color_on},
		{   1,		"off",		"turn off",					color_off},
		{   1,		"show", 	"show color",				color_show},
		{ 0,		"clear", 	"clear screen", 			clear_screen},
		{-1,		"clr", 		"", 						NULL},
		{-1,		"clrscr",	"", 						NULL},
		{ 0,		"scan", 	"nrf scan", 				nrf_scan},
};

#define microrl_actions_length (sizeof(microrl_actions)/sizeof(microrl_action_t))

// array for completion
char * compl_word [microrl_actions_length + 1];



typedef struct {
	microrl_color_e name;
	char code[10];
} microrl_color_t;

const microrl_color_t microrl_color_lookup [] =
{
		{C_NC,		COLOR_NC},
		{C_WHITE,	COLOR_WHITE},
		{C_BLACK,	COLOR_BLACK},
		{C_BLUE,	COLOR_BLUE},
		{C_L_BLUE,	COLOR_LIGHT_BLUE},
		{C_GREEN,	COLOR_GREEN},
		{C_L_GREEN,	COLOR_LIGHT_GREEN},
		{C_CYAN,	COLOR_CYAN},
		{C_L_CYAN,	COLOR_LIGHT_CYAN},
		{C_RED,		COLOR_RED},
		{C_L_RED,	COLOR_LIGHT_RED},
		{C_PURPLE,	COLOR_PURPLE},
		{C_L_PURPLE,COLOR_LIGHT_PURPLE},
		{C_BROWN,	COLOR_BROWN},
		{C_YELLOW,	COLOR_YELLOW},
		{C_GRAY,	COLOR_GRAY},
		{C_L_GRAY,	COLOR_LIGHT_GRAY}
};

#define microrl_color_lookup_length (sizeof(microrl_color_lookup)/sizeof(microrl_color_t))

const microrl_color_e microrl_help_color [] =
{
		C_GREEN,
		C_L_GREEN,
		C_PURPLE,
		C_L_PURPLE
};

#define microrl_help_color_lenght (sizeof(microrl_help_color)/sizeof(microrl_color_e))


/****************************************************************
 *
 */

void microrl_print_char(char buf)
{
	microrl_insert_char(p_mcrl, (int) buf);
}

void print (const char * str)
{
	if ((!color_out) && (str[0] == '\e')) // don't print escape characters
		return;
	uint16_t len = 0;
	while (str[++len] != 0);
	uint32_t timeout = HAL_GetTick();
	while (((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState!=0)
		if (HAL_GetTick() - timeout >= 5)
			break;
	CDC_Transmit_FS((uint8_t*)str, len);
}

int find_color_by_name(microrl_color_e color)
{
	for (int i = 0; i < microrl_color_lookup_length; i++)
	{
		if (microrl_color_lookup[i].name == color)
		{
			return i;
		}
	}
	return 0;
}

int print_color(const char * str, microrl_color_e color)
{
	print(microrl_color_lookup[find_color_by_name(color)].code);
	print(str);
	print(COLOR_NC);
	return 0;
}

int str_length(const char * str)
{
	int i = 0;
	while (str[i])
		i++;
	return i;
}


int print_help(int argc, const char * const * argv)
{
	print(_VER);
	print(ENDL);
	print ("Use ");
	print_color("TAB", C_GREEN);
	print(" key for completion");
	print (ENDL);
	print ("Available commands:");
	for (int i = 0; i < microrl_actions_length; i++)
	{
		if (microrl_actions[i].level == -1) // print synonyms
		{
			assert_param(i > 0);
			if (microrl_actions[i - 1].level != -1)
				print_color(" aka ", C_L_PURPLE);
			else
				print_color("/", C_L_PURPLE);
			print_color (microrl_actions[i].cmd, C_PURPLE);
		}
		else
		{
			print(ENDL);
			for (int e = -4; e < microrl_actions[i].level; e++)
				print(" ");
			print_color(microrl_actions[i].cmd, microrl_help_color[microrl_actions[i].level]);
			for (int e = 0; e < MICRORL_CMD_LENGTH + 2 -
								microrl_actions[i].level - str_length(microrl_actions[i].cmd); e++)
				print (" ");
			switch (microrl_actions[i].level){
			case 0:
				print ("-");
				break;
			case 1:
				print ("^");
				break;
			default:
				print ("#");
				break;
			}
			print (" ");
			print (microrl_actions[i].help_msg);
		}
	}
	print(ENDL);
	return 0;
}



int execute (int argc, const char * const * argv)
{
	int (*func)   (int argc, const char * const * argv ) = NULL;

	/*
	 * iterate throw levels and synonyms - run the func from the first (main) synonym
	 * run last found functions with all parameters - functions should check or ignore additional parameters
	 * if nothing found - show err msg
	 */

	int last_main_synonym = 0;
	int synonym_level = 0;
	bool tokens_found = false;
	for (int i = 0; i < argc; i++)
	{
		for (int n = last_main_synonym; n < microrl_actions_length; n++)
		{
			tokens_found = false;
			int current_level = microrl_actions[n].level;
			// next higher level command found, break;
			if (current_level != -1)
				synonym_level = current_level; // save the synonym level
			if ((current_level != -1) && (current_level < i))
				break;
			if (current_level == i)
				last_main_synonym = n;
			if ((strcmp(argv[i], microrl_actions[n].cmd) == 0) &&
					(i == synonym_level))
			{
				tokens_found = true;
				func = microrl_actions[last_main_synonym++].func;
				break;
			}
		}
		if (!tokens_found)	// nothing found, nothing to do here
			break;
	}

	if (func != NULL)
	{
		return func(argc, argv); // function found
	} else if (tokens_found)
	{
		print_color ("command: '", C_L_RED);
		print_color ((char*)argv[0], C_L_RED);
		print_color ("' needs additional arguments", C_L_RED);
		print(ENDL);
		print_color ("use '", C_NC);
		print_color ("?", C_GREEN);
		print_color ("' for help", C_NC);
		print (ENDL);
		return 1;
	}
	else
	{
		print_color ("command: '", C_RED);
		print_color ((char*)argv[0], C_RED);
		print_color ("' not found", C_RED);
		print(ENDL);
		print_color ("use '", C_NC);
		print_color ("?", C_GREEN);
		print_color ("' for help", C_NC);
		print (ENDL);
		return 1;

	}
}

#ifdef _USE_COMPLETE
//*****************************************************************************
// completion callback for microrl library
char ** complet (int argc, const char * const * argv)
{
	int j = 0;

	compl_word [0] = NULL;

	/*
	 * if no parameters - print all cmd with friend =="" && father == ""
	 * if parameter == 1 search with parent == ""
	 * if parameter > 1 search with parent == (parameter-2)
	 */

	/*
	 * print cmd and synonyms with level == argc-1.
	 * if argc == 0 print without synonyms
	 */

	if (argc == 0)
	{
		// if there is no token in cmdline, just print all available token
		for (int i = 0; i < microrl_actions_length; i++) {
			if (microrl_actions[i].level == 0)
			compl_word[j++] = (char*) microrl_actions [i].cmd;
		}
	} else {
		// get last entered token
		char * bit = (char*)argv [argc-1];
		// iterate through our available token and match it
		// based on previous tokens in the line, find the correct one shift
		int last_main_synonym = 0;
		int synonym_level = 0;
		bool tokens_found = false;
		for (int i = 0; i < argc; i++)
		{
			for (int n = last_main_synonym; n < microrl_actions_length; n++)
			{
				tokens_found = false;
				int current_level = microrl_actions[n].level;
				// next higher level command found, break;
				if (current_level != -1)
					synonym_level = current_level; // save the synonym level
				if ((current_level != -1) && (current_level < i))
					break;
				if (current_level == i)
					last_main_synonym = n;
				if ((i == argc - 1) && (strstr(microrl_actions [n].cmd, bit) == microrl_actions [n].cmd) &&
										(i == synonym_level))
				{
					tokens_found = true;
					compl_word [j++] =(char*) microrl_actions [n].cmd;
				}
				else if ((strcmp(argv[i], microrl_actions[n].cmd) == 0) && (i == synonym_level))
				{
					last_main_synonym++;
					tokens_found = true;
					break;
				}
			}
			if (!tokens_found)	// nothing found, nothing to do here
				break;
		}
	}

	// note! last ptr in array always must be NULL!!!
	compl_word [j] = NULL;
	// return set of variants
	return compl_word;
}
#endif

void init_microrl(void)
{
	  microrl_init(p_mcrl, print);
	  // set callback for execute
	  microrl_set_execute_callback (p_mcrl, execute);

	#ifdef _USE_COMPLETE
	  // set callback for completion
	  microrl_set_complete_callback (p_mcrl, complet);
	#endif
	  // set callback for Ctrl+C
	  microrl_set_sigint_callback (p_mcrl, sigint);
}


int clear_screen(int argc, const char * const * argv)
{
	print ("\033[2J");    // ESC seq for clear entire screen
	print ("\033[H");     // ESC seq for move cursor at left-top corner
	return 0;
}

int color_toggle 	(int argc, const char * const * argv)
{
	color_out ^= 1;
	return 0;
}

int color_on 		(int argc, const char * const * argv)
{
	color_out = 1;
	print_color ("Color output is ON", C_GREEN);
	print(ENDL);
	return 0;
}

int color_off 		(int argc, const char * const * argv)
{
	color_out = 0;
	print ("Color output is OFF");
	print(ENDL);
	return 0;
}

int color_show 		(int argc, const char * const * argv)
{
	if (color_out)
		print_color("Color output is ON", C_GREEN);
	else
		print ("Color output is OFF");
	print(ENDL);
	return 0;
}



