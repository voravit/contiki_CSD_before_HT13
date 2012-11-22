#ifndef __COMM_H__
#define __COMM_H__

#include <stdint.h>

//#define COMMAND_LINE_LENGTH 32
#define COMMAND_LINE_LENGTH 64
#define COMMAND_BUFFER_SIZE 100

//#define API_VERSION "0.1" Robert
#define API_VERSION "0.2 2011-08" //camilo

typedef struct
{
	char cmd[COMMAND_BUFFER_SIZE][COMMAND_LINE_LENGTH]; // commands
	volatile uint8_t index; // next empty place in the buffer
	volatile uint8_t pos; // index of the first non executed command
	volatile uint8_t history_index;
} command_buffer;



void comm_init();
uint8_t receive_byte(uint8_t input, char *output_buffer);
void unset_echo();
void set_echo();
int is_echo();
int is_whitespace(char chr);
void print(const char*);
void print_int(const int intBuf);
void print_float(const float *charBuf);
void welcomeMsg();

void break_float(float f, int *integer, int *decimal, int dlen, int base);

#endif

