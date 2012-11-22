#include <stdint.h>
#include <stdio.h>
#include <string.h>
//#include "../minne3ctrlr.h"
#include "LPC17xx.h"
#include "comm.h"
#include "commands.h"
//#include "../pwm/pwm.h"
#include "../init/config.h"
#include "../utils/utils.h"

#define END_OF_FILE "\x1A"
static char lastCommand[COMMAND_LINE_LENGTH];
static char *shell_str = "Querol> ";
static char *text_enter_debug_mode = "\r\nEntered debug mode";
static char *text_cmd_buff_full = "\r\nCommand buffer full. Cannot execute!\r\n";
static char text_backspace[3] = {0x8, 0x20, 0x8};

static char cmd_line[COMMAND_LINE_LENGTH];
static uint16_t cmd_line_pos = 0;
static int echo = 0;
static command_buffer cmd_buff;

static int up_flag=0;

extern void UART_init();
extern uint8_t uart_tx_std_write(char charBuf[]);

extern command command_list[NUMBER_OF_COMMANDS];
//extern uint32_t PWM_Init( uint32_t cycle);
//char *findParameter();

void comm_init()
{
	//UART channel could be 0 or 1, but 0 overlaps ADC1 and ADC2
	UART_init(UART_CHANNEL);
	cmd_buff.pos = 0;
	cmd_buff.index = 0;
	//print("test");
	init_commands();
}

/*
 * Prints non-formatted NULL-terminated string to console.
 */
uint8_t printchar(const char *charBuf) {
	/* Because UART0_IRQHandler does not work in contiki, we just print string to UART0 interupt register THR */ 
	uint16_t i = 0;
	while (charBuf[i] != '\0')
	{
		LPC_UART0->THR = charBuf[i];
		i++;
	}
	
	return 0;

	//return uart_tx_std_write((char*)charBuf);
}

void print(const char *charBuf) {
	if(echo==0)
	{
		//printchar("\033");
		//printchar("S1");
	}
	printchar(charBuf);
	if(echo==0)
	{
		printchar("\033E");
	}
}

uint8_t print_int_basic(const int charBuf) {
	char tmpstr1[1024];
	sprintf(tmpstr1, "%d", charBuf);
	return (int)uart_tx_std_write(tmpstr1);
}

void print_int(const int intBuf) {
	if(echo==0)
		{
		  printchar("\033");
		  printchar("S1");
		}
	print_int_basic(intBuf);
	if(echo==0)
	{
		printchar("\033E");
	}

}


void print_float(const float *charBuf) {
	char tmpstr[20]="";
	int integer, decimal;
	break_float(*charBuf, &integer, &decimal, 3, 10);
	sprintf(tmpstr, "%3d.%03d ", integer, decimal);
	char newStr[strlen(tmpstr)];
	print (strtrim(tmpstr,newStr));
	//return (int)uart_tx_std_write(floatToString(charBuf));
}
void break_float(float f, int *integer, int *decimal, int dlen, int base) {
	*integer = (int)f;
	f -= *integer;
	for(; dlen > 0; dlen--) {
		f *= base;
	}
	*decimal = (int)f;
//	*decimal = 0;
}

/*
 * Returns 1 if chr is whitespace of NULL-character. 0 otherwise.
 */
int is_whitespace(char chr) {
	switch(chr) {
		case '\0':
		case ' ':
		case '\r':
		case '\n':
		case '\t':
			return 1;
		default:
			return 0;
	}
}

/*
 * Returns 0 if the command strings given as arguments do not match
 * and 1 otherwise. Strings are compared to the first whitespace 
 * character.
 */
static int commands_match(char *cmd1, const char *cmd2) {
	int i;
	for(i=0; !is_whitespace(cmd1[i]) && !is_whitespace(cmd2[i]); i++) {
		if(cmd1[i] != cmd2[i]) 
			return 0;
	}
	return is_whitespace(cmd1[i]) && is_whitespace(cmd2[i]);
}

/*
 * Handles first command from the command buffer and executes it.
 */
void handle_command() {
	int i;
	int j;
	int r=0;
	int executed = 0;

	if(cmd_buff.index != cmd_buff.pos) {
		if(cmd_buff.cmd[cmd_buff.pos][0] == '\0') {
			executed = 1;
		}

	//if(!echo)
	//{
		for(j=0; j < 64; j++)
		{   if(cmd_buff.cmd[cmd_buff.pos][j]=='\123'&&cmd_buff.cmd[cmd_buff.pos][j+1]=='\061')
		    {
			  while(cmd_buff.cmd[cmd_buff.pos][j+2]!='\105')
			  {
				cmd_buff.cmd[cmd_buff.pos][r]=cmd_buff.cmd[cmd_buff.pos][j+2];
         		r++;j++;
			  }

		     }
		}

		 while(r<64&&r>0)
		{
				cmd_buff.cmd[cmd_buff.pos][r]='\0';
				r++;
		}

	//}

		for(i=0; i < NUMBER_OF_COMMANDS && !executed; i++) {
			if(commands_match(cmd_buff.cmd[cmd_buff.pos], command_list[i].command)) {
				command_list[i].command_function(cmd_buff.cmd[cmd_buff.pos]);
				executed = 1;
				if(echo) {
					print(END_OF_FILE);
					print("\r\n");
				} else {
					print(END_OF_FILE);
				}
				break;
			}
		}

		if(!executed) {

			if(echo)
			{
			print("Unknown command: ");
			print(cmd_buff.cmd[cmd_buff.pos]);
			print("\r\n");
			print(END_OF_FILE);
			}
			else
			{
				print("error,input again!");
				print("\r\n");
			}
		}

		cmd_buff.pos = (cmd_buff.pos + 1) % COMMAND_BUFFER_SIZE;

		if(cmd_buff.index == cmd_buff.pos && echo) {
			print(shell_str);
		}
	}
}


/*
 * This function is called when a character is received from UART.
 * This function handles command line editing and echoing if in 
 * debug mode. If '\r' is received, the command line is copied to
 * command buffer, from which the commands can later be executed.
 *
 * THIS FUNCTION MUST NOT WRITE MORE THAN 64BYTES TO THE output_buffer.
 * The output_buffer is meant only for short messages, like echoes.
 */
uint8_t receive_byte(uint8_t input, char *output_buffer) {
	int ii;
	// Execute command
	if(input == '\r') {
		cmd_line[cmd_line_pos] = '\0';
		cmd_line_pos = 0;

		// Check if there is room in the command buffer
		if( (cmd_buff.index + 1) % COMMAND_BUFFER_SIZE == cmd_buff.pos && echo) {
			// echo mode
			sprintf(output_buffer, text_cmd_buff_full);
			return strlen(text_cmd_buff_full);
		} else if( (cmd_buff.index + 1) % COMMAND_BUFFER_SIZE == cmd_buff.pos && !echo) {
			// non-echo mode
			return 0;
		}
		if(strlen(cmd_line)>1){//only saves if there is a last command
			strcpy(lastCommand, cmd_line);//save this command as the alst executred command//camilo
		}
		strncpy(cmd_buff.cmd[cmd_buff.index], cmd_line, COMMAND_LINE_LENGTH);
		cmd_buff.index = (cmd_buff.index + 1) % COMMAND_BUFFER_SIZE;
		cmd_buff.history_index = cmd_buff.index;

		if(echo) {
			sprintf(output_buffer, "\r\n");
			return strlen(output_buffer);
		} else {
			return 0;
		}
	}


	// Backspace
	if(input == 8 && cmd_line_pos > 0) {
		strcpy(output_buffer, text_backspace);
		cmd_line_pos--;
		return 3;
	}

	if(input==27)
		up_flag=1;
	else if(input==91 && up_flag==1)
	{
		up_flag=2;
		return 0;
	}
	else if(input==65 && up_flag==2 && lastCommand!="") {
	// last command (up arrow)
	//MAde by camilo!!! (it was difficult)
	//if(input == 91 &&lastCommand!="") {
		//strcpy(output_buffer, "hola");
		//print("exec");
		if(cmd_buff.history_index!=0)
			cmd_buff.history_index--;

		//print("aaaaaaaaaaaaaaaaaaaaaaaaa\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
		for(ii=0; ii<20; ii++)
			print(text_backspace);
		//print("\r\n");
		cmd_line_pos=strlen(cmd_buff.cmd[cmd_buff.history_index]);
		strcpy(cmd_line, cmd_buff.cmd[cmd_buff.history_index]);
		strncpy(output_buffer, cmd_line, strlen(cmd_line));

		return strlen(cmd_line);

		//this executes the last command but does not wait
//		strcpy(cmd_line, lastCommand);
//		strncpy(cmd_buff.cmd[cmd_buff.index], cmd_line, COMMAND_LINE_LENGTH);
//		cmd_buff.index = (cmd_buff.index + 1) % COMMAND_BUFFER_SIZE;
//		return strlen(output_buffer);
	}
	else if(input==66 && up_flag==2 && lastCommand!="") {
		// last command (up arrow)
		//MAde by camilo!!! (it was difficult)
		//if(input == 91 &&lastCommand!="") {
		//strcpy(output_buffer, "hola");
		//print("exec");
		//print("aaaaaaaaaaaaaaaaaaaaaaaaa\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
		for(ii=0; ii<20; ii++)
			print(text_backspace);
		//printchar("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
		//print("\r\n");
//		print("\r\n**********************************\r\n");
//		print("cmd_buff.pos = ");print_int(cmd_buff.pos);print("\r\n");
//		print("cmd_buff.index = ");print_int(cmd_buff.index);print("\r\n");
//		print("cmd_buff.history_index = ");print_int(cmd_buff.history_index);print("\r\n");
//		print("\r\n----------------------------------\r\n");
		if(cmd_buff.history_index<cmd_buff.index)
		{
			cmd_buff.history_index++;
//			print_int(cmd_buff.history_index);
		}
		cmd_line_pos=strlen(cmd_buff.cmd[cmd_buff.history_index]);
		strcpy(cmd_line, cmd_buff.cmd[cmd_buff.history_index]);
		strncpy(output_buffer, cmd_line, strlen(cmd_line));

		return strlen(cmd_line);
	}
	else
		up_flag=0;

	// Do not exceed the command buffer size
	if(cmd_line_pos >= COMMAND_LINE_LENGTH - 1) {
		return 0;
	}

	// Echo back characters between 0x20 and 0x7E if in debug mode
	if(input >= 0x20 && input <= 0x7E) { 
		cmd_line[cmd_line_pos++] = input;
		
		if(echo) {
			output_buffer[0] = input;
			return 1;
		} else {
			return 0;
		}
	}

	return 0;
}

void welcomeMsg()
{
	echo = 1;
	print("-------------- _____               _   _      _   --------------\r\n");
	print("--------------|  __ \\             | \\ | |    | |  --------------\r\n");
	print("--------------| |  \\/_ __ ___  ___|  \\| | ___| |_ --------------\r\n");
	print("--------------| | __| '__/ _ \\/ _ \\ . ` |/ _ \\ __|--------------\r\n");
	print("--------------| |_\\ \\ | |  __/  __/ |\\  |  __/ |_ --------------\r\n");
	print("-------------- \\____/_|  \\___|\\___\\_| \\_/\\___|\\__|--------------\r\n");
	print("------------Welcome to the GreeNet configuration menu-----------\r\n");
	print("----------Enter \'help\' for a list of supported commands---------\r\n");
	echo = 0;
}

void set_echo() {
	echo = 1;
	print(text_enter_debug_mode);
}

void unset_echo() {
	echo = 0;
}

int is_echo() {
	return echo;
}



