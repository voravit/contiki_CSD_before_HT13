#ifndef __UART_H__
#define __UART_H__


#include <stdint.h>
#define FIFO_SIZE 16
//#define UART_BUFFER_SIZE 256
#define UART_BUFFER_SIZE 1024 //expanded by camilo

/* UART buffer 
 */
typedef struct
{
	uint8_t rx_buf[UART_BUFFER_SIZE]; // receive buffer
	uint8_t tx_buf[UART_BUFFER_SIZE]; // transmit buffer
	volatile uint16_t rx_start_index;	// the index of starting index for received byte
	volatile uint16_t rx_len;	// the number of bytes in the receiving buffer
	volatile uint16_t tx_start_index;   // the index of starting byte for transmitting
	volatile uint16_t tx_len;   // the number of bytes in the transmitting buffer
//	volatile uint8_t tx_fifo_avail; // if there are places in the transmitting fifo
} uart_buffer_type;

/* init for UART0/1/2/3 */
void UART_init(int uart_num);
void UART0_init();
void UART1_init();
void UART2_init();
void UART3_init();

/* Interrupt handlers for UART0/1/2/3 */
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);

/* init all the member of the uart buffer 
 * @param	*buff: the buffer struture to init
 */
void uart_buf_init(uart_buffer_type *buff);

/* write data to the transmitting fifo 
 * change this function for different behaviours
 * @param	*buff	a pointer to the data structer that contain the transmitting and receiving buffer
 * @param	number_of_byte	the number of byte to send 
 */
void uart_send(uart_buffer_type *buff, uint8_t number_of_byte, uint8_t uart_num);

/* Write string to the transmitting buffer 
 * @param	charBuff[]: string to write to the buffer
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if sucessful
 *		1 if the buffer is full
 */
uint8_t uart_tx_buf_write(char charBuf[], uart_buffer_type *buff); 

/* Write byte ch to the transmitting buffer 
 * @param	ch: byte to write to the buffer
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if sucessful
 *		1 if the buffer is full
 */
uint8_t uart_tx_buf_write_byte(uint8_t ch, uart_buffer_type *buff);

/* Read byte from the receiving buffer. The character will be deleted from the receiving buffer.
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if no character. So, in order to use the function, may be should check the length of the receiving buffer first
 *			in case the buffer can contains '\0';
 *		the first character in the buffer
 */
 
uint8_t uart_rx_buf_read_byte(uart_buffer_type *buff);

/* Read number of bytes from the receiving buffer. The characters will be deleted from the receiving buffer.
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @param	*result: a pointer to a place to contain the results
 * @param	number_of_byte: the number of bytes to read
 * @return	the number of character that actually read.
 */
 
uint8_t uart_rx_buf_read(uart_buffer_type *buff, uint8_t *result, uint8_t number_of_byte);

/* Write byte ch to the receiving buffer 
 * @param	ch: byte to write to the buffer
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if sucessful
 *		1 if the buffer is full
 */
 
uint8_t uart_rx_buf_write_byte(uint8_t ch, uart_buffer_type *buff);

/* Write string to the receiving buffer 
 * @param	charBuff[]: string to write to the buffer
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if sucessful
 *		1 if the buffer is full
 */

uint8_t uart_rx_buf_write(char charBuf[], uart_buffer_type *buff);


#endif
