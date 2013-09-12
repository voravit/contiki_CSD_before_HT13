/*
 * control.h
 *
 *  Created on: 08/03/2012
 *      Author: Jorge Querol
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
