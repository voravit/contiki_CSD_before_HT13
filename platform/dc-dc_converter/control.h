/*
 * Copyright (c) 2013, KTH, Royal Institute of Technology(Stockholm, Sweden)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * From the original control.h file
 * Author: Jorge Queroal
 * Created on: 2012-08-03
 *
 * Ported to contiki by: Voravit Tanyingyong
 *
 */

#ifndef CONTROL_H_
#define CONTROL_H_

//Declare the name of the controller process
PROCESS_NAME(bang_control_process);

#define Vdd 3.293
#define I_IMAX	1
#define I_VMAX	25
#define UART_BUFFER_SIZE 1024

/* UART buffer 
 */
typedef struct
{
        uint8_t rx_buf[UART_BUFFER_SIZE]; // receive buffer
        uint8_t tx_buf[UART_BUFFER_SIZE]; // transmit buffer
        volatile uint16_t rx_start_index;       // the index of starting index for received byte
        volatile uint16_t rx_len;       // the number of bytes in the receiving buffer
        volatile uint16_t tx_start_index;   // the index of starting byte for transmitting
        volatile uint16_t tx_len;   // the number of bytes in the transmitting buffer
//      volatile uint8_t tx_fifo_avail; // if there are places in the transmitting fifo
} uart_buffer_type;

//#define ShowPeriodically

typedef enum
{
        VREF = 0,
        IMAX,
        VMAX,
}ctrl_params_t;

typedef enum
{
        VOUT = 0,
        VIN,
        IOUT,
        IIN,
}svector_t;

char* strtrim(char *str, char *newStr);
void break_float(float f, int *integer, int *decimal, int dlen, int base);
void print_float(const float *charBuf);

int get_user_allowed(void);
void set_user_allowed(int allowed);
char* get_converter_state(void);
int set_ctrl_params(ctrl_params_t var, float value);
float get_ctrl_params(ctrl_params_t var);
float get_svector(svector_t var);


void ValueInit(void);
//void BangBang(void);
void MeanValues(void);

#endif /* CONTROL_H_ */
