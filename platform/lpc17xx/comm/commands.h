/*
 * commands.h
 *
 *  Created on: 15/03/2012
 *      Author: Jorge Querol
 */

#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#define NUMBER_OF_COMMANDS 7

typedef struct
{
	void (*command_function)(char*);
	const char *command_desc;
	const char *command;
}command;

void init_commands(void);

#endif
