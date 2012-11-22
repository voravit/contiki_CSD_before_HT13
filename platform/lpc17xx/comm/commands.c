/*
 * commands.c
 *
 *  Created on: 15/03/2012
 *      Author: Jorge Querol
 */

#include <stdlib.h>
#include "../LPC17xx.h"
#include "../control/control.h"
#include "../comm/comm.h"
#include "../comm/commands.h"

command command_list[NUMBER_OF_COMMANDS];

extern int Vref, Imax, Vmax;
extern long Vo, Vi, Io, Ii, Iop;
extern float E;

void cmd_help(char *cmd);
void cmd_SetVref(char *cmd);
void cmd_SetImax(char *cmd);
void cmd_SetVmax(char *cmd);
void cmd_ShowParams(char *cmd);
void cmd_ShowValues(char *cmd);
void cmd_repeat(char *cmd);

void init_commands(void)
{
	int i = 0;

	command_list[i].command_function = cmd_help;
	command_list[i].command_desc = "This list of supported commands";
	command_list[i++].command = "help";

	command_list[i].command_function = cmd_SetVref;
	command_list[i].command_desc = "Set Vref value";
	command_list[i++].command = "vref";

	command_list[i].command_function = cmd_SetImax;
	command_list[i].command_desc = "Set Imax value";
	command_list[i++].command = "imax";

	command_list[i].command_function = cmd_SetVmax;
	command_list[i].command_desc = "Set Vmax value";
	command_list[i++].command = "vmax";

	command_list[i].command_function = cmd_ShowParams;
	command_list[i].command_desc = "Show Parameters";
	command_list[i++].command = "par";

	command_list[i].command_function = cmd_ShowValues;
	command_list[i].command_desc = "Show Mean Values";
	command_list[i++].command = "val";

	command_list[i].command_function = cmd_repeat;
	command_list[i].command_desc = "Repeat last command";
	command_list[i++].command = "[UP ARROW]";
}

void cmd_help(char *cmd)
{
	int i;

	for (i = 0; i < NUMBER_OF_COMMANDS; i++)
	{
		print(command_list[i].command);
		print("\t");
		print(command_list[i].command_desc);
		print("\r\n");
	}
}

void cmd_SetVref(char *cmd)
{
	int i = 0;
	float aux;

	// Find first space character. State should be followed by it
	while(cmd[i] != ' ' && cmd[i] != '\0') {
		i++;
	}

	aux = atoff(cmd + i);
	aux = (aux * 4095 * 3) /(Vdd * 28);
	if((aux >= 0) && (aux <= Vmax))
		Vref = (int)aux;
}

void cmd_SetImax(char *cmd)
{
	int i = 0;
	float aux;

	// Find first space character. State should be followed by it
	while(cmd[i] != ' ' && cmd[i] != '\0') {
		i++;
	}

	aux = atoff(cmd + i);
	if((aux >= 0) && (aux <= 6))
		Imax = (int)(aux * 4095 * 0.151) / (Vdd * 2);
}

void cmd_SetVmax(char *cmd)
{
	int i = 0;
	float aux;

	// Find first space character. State should be followed by it
	while(cmd[i] != ' ' && cmd[i] != '\0') {
		i++;
	}

	aux = atoff(cmd + i);
	if((aux >= 0) && (aux <= 30))
		Vmax = (int)(aux * 4095 * 3) / (Vdd * 28);
}

void cmd_ShowParams(char *cmd)
{
	float aux;
	print("\t\t Vref:");
	aux = (Vref * Vdd * 28) / (4095 * 3);
	print_float(&aux);
	print(" Imax:");
	aux = (Imax * Vdd * 2) / (4095 * 0.151);
	print_float(&aux);
	print(" Vmax:");
	aux = (Vmax * Vdd * 28) / (4095 * 3);
	print_float(&aux);
}

void cmd_ShowValues(char *cmd)
{
	float aux;
	print("\t\t Vo:");
	aux = (Vo * Vdd * 28) / (4095 * 3);
	print_float(&aux);
	print(" Vin:");
	aux = (Vi * Vdd * 28) / (4095 * 3);
	print_float(&aux);
	print(" Io:");
	aux = (Io * Vdd * 2) / (4095 * 0.151);
	print_float(&aux);
	print(" Ii:");
	aux = (Ii * Vdd * 2) / (4095 * 0.151);
	print_float(&aux);
}

void cmd_repeat(char *cmd){}
