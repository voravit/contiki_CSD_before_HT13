/* UART 
 * Using types and registers definition from LPC17xx.h 
 * (CMSIS Cortex-M3 Core Peripheral Access Layer Header File for
 *           NXP LPC17xx Device Series)
 */

#include "../LPC17xx.h"
#include "../uart/uart.h"
//#include "../temp/temp_mon.h"
#include "../init/config.h"

extern uint8_t receive_byte(uint8_t input, char *output_buffer);
//extern uint8_t read_temps(temp_sensor_t *temp);

uint8_t welcome_msg[] = "\r";
uint8_t is_start = 1;
//extern temp_sensor_t temp_sensor;

/* received and transmitted data buffers */
static uart_buffer_type uart_buf;
	
void UART_init(int uart_num)
{
	switch(uart_num)
	{
	case 0:
		UART0_init();
		break;
	case 1:
		UART1_init();
		break;
	case 2:
		UART2_init();
		break;
	case 3:
		UART3_init();
		break;
	default:
		return;
	}
}

/* init for UART0 */
void UART0_init()
{
	uint32_t tmp;
	/* Disable UART0 interrupt */
	NVIC_DisableIRQ(UART0_IRQn);
	/* init the buffer */
	uart_buf_init(&uart_buf);
	/* Enable. 
	 * UART0 bit 3, UART1 bit 4, UART2 bit 24, UART3 bit 25
	 * UART0 and UART1 enable when reset.
	 * 0 = disable, 1 = enable
	 */
	LPC_SC->PCONP |= (1 << 3);
	/* Select pin function 
	 * p0.2 = txd0, p0.3 = rxd0
	 * function = 01, bit = 7:6, 5:4
	 */
	LPC_PINCON->PINSEL0 &= ~0x000000F0;
	LPC_PINCON->PINSEL0 |= 0x00000050;
	
	/* Select pin mode 
	 * Not enable pull-down resistors on receive pins
	 * Normal mode, not open drain
	 */
	LPC_PINCON->PINMODE0 &= ~0x000000F0;
	LPC_PINCON->PINMODE_OD0 &= ~0x0000000C;

	/* Set clock
	 * The internal clock is used here 100 MHz
	 * UART0 7:6, UART1 9:8
	 * 00 = cclk/4, 01 = cclk, 10 = cclk/2, 11 = cclk/8
	 * Let set it to cclk
	 */
	LPC_SC->PCLKSEL0 &= ~0x00000080;
	LPC_SC->PCLKSEL0 |= 0x00000040;
	
	/* Set transmitted character format
	 * 8 bit, no parity, 1 stop bit
	 * also enable DLAB bit for DLL and DLM access
	 */
	LPC_UART0->LCR = 0x83;
	/* Setting the baudrate 
	 * let it be 115200
	 * For calculation, see the lpc17xx manual
	 * We get DLL = 37, DLM = 0, DIVADDVAL = 7, MULVAL = 15
	 * DIVADDVAL and MULVAL are set via FDR register
	 */
	LPC_UART0->DLL = 37;
	LPC_UART0->DLM = 0;
	LPC_UART0->FDR = 0xF7;
	
	/* Enabling the FIFOs and reseting them
	 */
	LPC_UART0->FCR = 0x07;
	 
	/* Enable interrupt
	 * Clear line status
	 * Enable RBR and RX Line Status
	 * Not enable THRE here
	 * have to set DLAB to 0
	 */
	tmp = LPC_UART0->LSR;
	LPC_UART0->LCR = 0x03;
	LPC_UART0->IER = 0x05;
	/* enable UART interrupt in NVIC
	 * using function from core_cm3.h
	 * interrupt type is from lpc17xx.h
	 */
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_SetPriority(UART0_IRQn, 3);
	is_start = 1;
}

/* init for UART1 */
void UART1_init()
{
	uint32_t tmp;
	/* Disable UART1 interrupt */
	NVIC_DisableIRQ(UART1_IRQn);
	/* init the buffer */
	uart_buf_init(&uart_buf);
	/* Enable.
	 * UART0 bit 3, UART1 bit 4, UART2 bit 24, UART3 bit 25
	 * UART0 and UART1 enable when reset.
	 * UART2 and UART3 disable when reset.
	 * 0 = disable, 1 = enable
	 */
	LPC_SC->PCONP |= (1 << 4);
	/* Select pin function
	 * p0.2 = txd0, p0.3 = rxd0
	 * p2.0 = txd1, p2.1 = rxd1
	 * p2.8 = txd2, p2.9 = rxd2
	 * p0.25 = txd3, p0.26 = rxd3
	 */
	LPC_PINCON->PINSEL4 &= ~0x0000000;
	LPC_PINCON->PINSEL4 |= 0x0000000A;

	/* Select pin mode
	 * Not enable pull-down resistors on receive pins
	 * Normal mode, not open drain
	 */
	LPC_PINCON->PINMODE4 &= ~0x0000000F;
	LPC_PINCON->PINMODE_OD2 &= ~0x00000002;

	/* Set clock
	 * The internal clock is used here ~ 4Mhz
	 * UART0 7:6, UART1 9:8
	 * UART2 17:16, UART3 19:18
	 * 00 = cclk/4, 01 = cclk, 10 = cclk/2, 11 = cclk/8
	 * Let set it to cclk/4
	 */
	LPC_SC->PCLKSEL0 &= ~0x00000300;

	/* Set transmitted character format
	 * 8 bit, no parity, 1 stop bit
	 * also enable DLAB bit for DLL and DLM access
	 */
	LPC_UART1->LCR = 0x83;
	/* Setting the baudrate
	 * let it be 9600
	 * For calculation, see the lpc17xx manual
	 * We get DLL = 4, DLM = 0, DIVADDVAL = 5, MULVAL = 8
	 * DIVADDVAL and MULVAL are set via FDR register
	 */
	LPC_UART1->DLL = 4;
	LPC_UART1->DLM = 0;
	LPC_UART1->FDR = 0x85;

	/* Enabling the FIFOs and reseting them
	 */
	LPC_UART1->FCR = 0x07;

	/* Enable interrupt
	 * Clear line status
	 * Enable RBR, THRE and RX Line Status
	 * have to set DLAB to 0
	 */
	tmp = LPC_UART1->LSR;
	LPC_UART1->LCR = 0x03;
	LPC_UART1->IER = 0x05;
	/* enable UART interrupt in NVIC
	 * using function from core_cm3.h
	 * interrupt type is from lpc17xx.h
	 */
	NVIC_EnableIRQ(UART1_IRQn);
	NVIC_SetPriority(UART1_IRQn, 3);
	is_start = 1;
}

/* init for UART2 */
void UART2_init()
{
	uint32_t tmp;
	/* Disable UART2 interrupt */
	NVIC_DisableIRQ(UART2_IRQn);
	/* init the buffer */
	uart_buf_init(&uart_buf);
	/* Enable.
	 * UART0 bit 3, UART1 bit 4, UART2 bit 24, UART3 bit 25
	 * UART0 and UART1 enable when reset.
	 * UART2 and UART3 disable when reset.
	 * 0 = disable, 1 = enable
	 */
	LPC_SC->PCONP |= (1 << 24);
	/* Select pin function
	 * p0.2 = txd0, p0.3 = rxd0
	 * p2.0 = txd1, p2.1 = rxd1
	 * p2.8 = txd2, p2.9 = rxd2
	 * p0.25 = txd3, p0.26 = rxd3
	 */
	LPC_PINCON->PINSEL4 &= ~0x000F0000;
	LPC_PINCON->PINSEL4 |= 0x000A0000;

	/* Select pin mode
	 * Not enable pull-down resistors on receive pins
	 * Normal mode, not open drain
	 */
	LPC_PINCON->PINMODE4 &= ~0x000F0000;
	LPC_PINCON->PINMODE_OD2 &= ~0x00000300;

	/* Set clock
	 * The internal clock is used here ~ 4Mhz
	 * UART0 7:6, UART1 9:8
	 * UART2 17:16, UART3 19:18
	 * 00 = cclk/4, 01 = cclk, 10 = cclk/2, 11 = cclk/8
	 * Let set it to cclk/4
	 */
	LPC_SC->PCLKSEL1 &= ~0x00018000;

	/* Set transmitted character format
	 * 8 bit, no parity, 1 stop bit
	 * also enable DLAB bit for DLL and DLM access
	 */
	LPC_UART2->LCR = 0x83;
	/* Setting the baudrate
	 * let it be 9600
	 * For calculation, see the lpc17xx manual
	 * We get DLL = 4, DLM = 0, DIVADDVAL = 5, MULVAL = 8
	 * DIVADDVAL and MULVAL are set via FDR register
	 */
	LPC_UART2->DLL = 4;
	LPC_UART2->DLM = 0;
	LPC_UART2->FDR = 0x85;

	/* Enabling the FIFOs and reseting them
	 */
	LPC_UART2->FCR = 0x07;

	/* Enable interrupt
	 * Clear line status
	 * Enable RBR, THRE and RX Line Status
	 * have to set DLAB to 0
	 */
	tmp = LPC_UART2->LSR;
	LPC_UART2->LCR = 0x03;
	LPC_UART2->IER = 0x05;
	/* enable UART interrupt in NVIC
	 * using function from core_cm3.h
	 * interrupt type is from lpc17xx.h
	 */
	NVIC_EnableIRQ(UART2_IRQn);
	NVIC_SetPriority(UART2_IRQn, 3);
	is_start = 1;
}

/* init for UART3 */
void UART3_init()
{
	uint32_t tmp;
	/* Disable UART3 interrupt */
	NVIC_DisableIRQ(UART3_IRQn);
	/* init the buffer */
	uart_buf_init(&uart_buf);
	/* Enable.
	 * UART0 bit 3, UART1 bit 4, UART2 bit 24, UART3 bit 25
	 * UART0 and UART1 enable when reset.
	 * UART2 and UART3 disable when reset.
	 * 0 = disable, 1 = enable
	 */
	LPC_SC->PCONP |= (1 << 24);
	/* Select pin function
	 * p0.2 = txd0, p0.3 = rxd0
	 * p2.0 = txd1, p2.1 = rxd1
	 * p2.8 = txd2, p2.9 = rxd2
	 * p0.25 = txd3, p0.26 = rxd3
	 */
	LPC_PINCON->PINSEL4 &= ~0x000F0000;
	LPC_PINCON->PINSEL4 |= 0x000A0000;

	/* Select pin mode
	 * Not enable pull-down resistors on receive pins
	 * Normal mode, not open drain
	 */
	LPC_PINCON->PINMODE4 &= ~0x000F0000;
	LPC_PINCON->PINMODE_OD2 &= ~0x00000300;

	/* Set clock
	 * The internal clock is used here ~ 4Mhz
	 * UART0 7:6, UART1 9:8
	 * UART2 17:16, UART3 19:18
	 * 00 = cclk/4, 01 = cclk, 10 = cclk/2, 11 = cclk/8
	 * Let set it to cclk/4
	 */
	LPC_SC->PCLKSEL1 &= ~0x00018000;

	/* Set transmitted character format
	 * 8 bit, no parity, 1 stop bit
	 * also enable DLAB bit for DLL and DLM access
	 */
	LPC_UART3->LCR = 0x83;
	/* Setting the baudrate
	 * let it be 9600
	 * For calculation, see the lpc17xx manual
	 * We get DLL = 4, DLM = 0, DIVADDVAL = 5, MULVAL = 8
	 * DIVADDVAL and MULVAL are set via FDR register
	 */
	LPC_UART3->DLL = 4;
	LPC_UART3->DLM = 0;
	LPC_UART3->FDR = 0x85;

	/* Enabling the FIFOs and reseting them
	 */
	LPC_UART3->FCR = 0x07;

	/* Enable interrupt
	 * Clear line status
	 * Enable RBR, THRE and RX Line Status
	 * have to set DLAB to 0
	 */
	tmp = LPC_UART3->LSR;
	LPC_UART3->LCR = 0x03;
	LPC_UART3->IER = 0x05;
	/* enable UART interrupt in NVIC
	 * using function from core_cm3.h
	 * interrupt type is from lpc17xx.h
	 */
	NVIC_EnableIRQ(UART3_IRQn);
	NVIC_SetPriority(UART3_IRQn, 3);
	is_start = 1;
}

/* Interrupt handler for UART0 */
void UART0_IRQHandler(void)
{
	uint8_t IIR_reg;
	uint8_t LSR_reg;
	uint8_t RBR_reg;
	
	/* interrupt identification - bit 3:1
	 * 011 - Receive Line Status - reset with UnLSR read
	 * 010 - Receive Data Available - reset with UnRBR read or FIFO drops below trigger level
	 * 110 - Character Time-out - reset with UnRBR read
	 * 001 - THRE Interrupt - reset with UnIIR Read (if it's the source) or THR write
	 * 1000: Rx data available
	 */
	 
	 IIR_reg = 0x0F & LPC_UART0->IIR;
	 switch (IIR_reg)
	 {
		/* If there is non-transmitted data to be put to fifo */
		case 0x01:
			if(uart_buf.tx_len <= 0) {
				LPC_UART0->IER = 0x05;
			} else {
				uart_send(&uart_buf, 16, 0);
			}
			break;

	 	 /* enable writing to the transmitting fifo */
	 	 case 0x02:
	 	 case 0x03:
	 	 	 /* if there are two character -> no interrupt 
	 	 	  * if there are no character in the transmitting buffer, disable it
	 	 	  */
			 uart_send(&uart_buf, 14, 0);
	 	 	 if (uart_buf.tx_len <= 0)
	 	 	 	LPC_UART0->IER = 0x05;
	 	 	 break;

	 	 /* read the available data to the read buffer */
	 	 case 0x04:	 	 	
	 	 case 0x05:	 	 	
			/* Switch led */
			//LPC_GPIO1->FIOPIN ^= (1 << 29);
	 	 	
	 	 	/* write character to the transmitter FIFO to init THRE */
		 	if (is_start)
		 	{
		 	 	uart_send(&uart_buf, 16, 0);
		 	 	is_start = 0;
		 	}
			 /* read one character to clear the interrupt */
			 RBR_reg = LPC_UART0->RBR;
			 uart_rx_buf_write_byte(RBR_reg, &uart_buf);
			 

			 
			 /* read more character */			 
			 while (LPC_UART0->LSR & 0x01)
	 	 	 {
	 	 	 	 /* if there is available place */
	 	 	 	 if (uart_buf.rx_len < UART_BUFFER_SIZE)
		 	 	 {
		 	 	 	RBR_reg = LPC_UART0->RBR;
					uart_rx_buf_write_byte(RBR_reg, &uart_buf);
					/* read temperature if enter*/

				 }
		 	 	 else
		 	 	 	break;
		 	 }

		 	 /* re-enable the THRE in case it's off */
		 	 LPC_UART0->IER = 0x07;	 	 
		 	 break;

	 	/* Receive Line Error */ 
		case 0x06:
		case 0x07:
			/* Switch led */
			//LPC_GPIO1->FIOPIN ^= (1 << 29);
			
			LSR_reg = LPC_UART0->LSR;
	 	 	/* Read if there are places in the buffer and there is data*/
			if ((LSR_reg & 0x01) && (uart_buf.rx_len < UART_BUFFER_SIZE))
			{
				RBR_reg = LPC_UART0->RBR;
				uart_rx_buf_write_byte(RBR_reg, &uart_buf);

			}
	 	 	/* do nothing with the receive line error */			
			break;	 	 	 

		/* Character time out */
	 	 case 0x0C:
	 	 case 0x0D:
			 /* read one character to clear the interrupt */
			 RBR_reg = LPC_UART0->RBR;
			uart_rx_buf_write_byte(RBR_reg, &uart_buf);

			 /* read more character */			 
		 	 while (LPC_UART0->LSR & 0x01)
	 	 	 {
				RBR_reg = LPC_UART0->RBR;
				uart_rx_buf_write_byte(RBR_reg, &uart_buf);
				/* read temperature if enter */
/*			 	 if (RBR_reg == '\r')
			 	 {
			 	 	temp_sensor.is_avail =  onewire_read_rom(temp_sensor.id);
			 	 	if (temp_sensor.is_avail == 0)
				 	 	temp_sensor.is_avail = read_temp(temp_sensor.scratchpad);
			 	 }
*/			 }
		 	 /* re-enable the THRE in case it's off */
		 	 LPC_UART0->IER = 0x07;
		 	 break;
	 }
}

#if 0
/* Interrupt handler for UART1 */
void UART1_IRQHandler(void)
{
	uint8_t IIR_reg;
			uint8_t LSR_reg;
			uint8_t RBR_reg;

			/* interrupt identification - bit 3:1
			 * 011 - Receive Line Status - reset with UnLSR read
			 * 010 - Receive Data Available - reset with UnRBR read or FIFO drops below trigger level
			 * 110 - Character Time-out - reset with UnRBR read
			 * 001 - THRE Interrupt - reset with UnIIR Read (if it's the source) or THR write
			 * 1000: Rx data available
			 */

			 IIR_reg = 0x0F & LPC_UART1->IIR;
			 switch (IIR_reg)
			 {
				/* If there is non-transmitted data to be put to fifo */
				case 0x01:
					if(uart_buf.tx_len <= 0) {
						LPC_UART1->IER = 0x05;
					} else {
						uart_send(&uart_buf, 16, 1);
					}
					break;

			 	 /* enable writing to the transmitting fifo */
			 	 case 0x02:
			 	 case 0x03:
			 	 	 /* if there are two character -> no interrupt
			 	 	  * if there are no character in the transmitting buffer, disable it
			 	 	  */
					 uart_send(&uart_buf, 14, 1);
			 	 	 if (uart_buf.tx_len <= 0)
			 	 	 	LPC_UART1->IER = 0x05;
			 	 	 break;

			 	 /* read the available data to the read buffer */
			 	 case 0x04:
			 	 case 0x05:
					/* Switch led */
					//LPC_GPIO1->FIOPIN ^= (1 << 29);

			 	 	/* write character to the transmitter FIFO to init THRE */
				 	if (is_start)
				 	{
				 	 	uart_send(&uart_buf, 16, 1);
				 	 	is_start = 0;
				 	}
					 /* read one character to clear the interrupt */
					 RBR_reg = LPC_UART1->RBR;
					 uart_rx_buf_write_byte(RBR_reg, &uart_buf);



					 /* read more character */
					 while (LPC_UART1->LSR & 0x01)
			 	 	 {
			 	 	 	 /* if there is available place */
			 	 	 	 if (uart_buf.rx_len < UART_BUFFER_SIZE)
				 	 	 {
				 	 	 	RBR_reg = LPC_UART1->RBR;
							uart_rx_buf_write_byte(RBR_reg, &uart_buf);

						 }
				 	 	 else
				 	 	 	break;
				 	 }

				 	 /* re-enable the THRE in case it's off */
				 	 LPC_UART1->IER = 0x07;
				 	 break;

			 	/* Receive Line Error */
				case 0x06:
				case 0x07:
					/* Switch led */
					//LPC_GPIO1->FIOPIN ^= (1 << 29);

					LSR_reg = LPC_UART1->LSR;
			 	 	/* Read if there are places in the buffer and there is data*/
					if ((LSR_reg & 0x01) && (uart_buf.rx_len < UART_BUFFER_SIZE))
					{
						RBR_reg = LPC_UART1->RBR;
						uart_rx_buf_write_byte(RBR_reg, &uart_buf);

					}
			 	 	/* do nothing with the receive line error */
					break;

				/* Character time out */
			 	 case 0x0C:
			 	 case 0x0D:
					 /* read one character to clear the interrupt */
					 RBR_reg = LPC_UART1->RBR;
					uart_rx_buf_write_byte(RBR_reg, &uart_buf);

					 /* read more character */
				 	 while (LPC_UART1->LSR & 0x01)
			 	 	 {
						RBR_reg = LPC_UART1->RBR;
						uart_rx_buf_write_byte(RBR_reg, &uart_buf);

					 }
				 	 /* re-enable the THRE in case it's off */
				 	 LPC_UART1->IER = 0x07;
				 	 break;
			 }
}
#endif

/* Interrupt handler for UART2 */
void UART2_IRQHandler(void)
{
}

/* init all the member of the uart buffer 
 * @param	*buff: the buffer struture to init
 */
void uart_buf_init(uart_buffer_type *buff)
{
	uint16_t i;
	i = 0;
	/* copy the welcome message to the transmit buffer */
	while (welcome_msg[i] != '\0')
	{
		buff->tx_buf[i] = welcome_msg[i];
		i++;
	}
	buff->rx_start_index = 0;
	buff->rx_len = 0;
	buff->tx_start_index = 0;
	buff->tx_len = i;
}

/**
 * 
 */
uint8_t uart_tx_std_write(char charBuf[]) {
	int val;
	if(charBuf[0] == '\0')
		return 0;
	val = uart_tx_buf_write(charBuf, &uart_buf);
	LPC_UART0->IER = 0x07;
	return val;
}


/* write data to the transmitting fifo 
 * change this function for different behaviours
 */
void uart_send(uart_buffer_type *buff, uint8_t number_of_byte, uint8_t uart_num)
{
	uint16_t tmp;
	uint8_t byte, len;
	char buffer[65];

	while(buff->rx_len > 0 && (UART_BUFFER_SIZE - buff->tx_len) >= 64) {
		byte = uart_rx_buf_read_byte(buff);
		len = receive_byte(byte, buffer);
		if(len != 0) {
			buffer[len] = '\0';
			uart_tx_buf_write(buffer, buff);
		} else {
			tmp = (buff->tx_len + buff->tx_start_index) % UART_BUFFER_SIZE;
			buff->tx_buf[tmp] = 0;
			buff->tx_len++;
		}
	}

	/* copy bytes to the FIFO */
	for (tmp = 0; tmp < number_of_byte; tmp++)
	{
		if (buff->tx_len <= 0)
			break;
		switch(uart_num)
		{
		case 0:
			LPC_UART0->THR = buff->tx_buf[buff->tx_start_index];
			break;
		case 1:
			LPC_UART1->THR = buff->tx_buf[buff->tx_start_index];
			break;
		case 2:
			LPC_UART2->THR = buff->tx_buf[buff->tx_start_index];
			break;
		case 3:
			LPC_UART3->THR = buff->tx_buf[buff->tx_start_index];
			break;
		default:
			break;
		}
		buff->tx_start_index++;
		buff->tx_start_index = buff->tx_start_index % UART_BUFFER_SIZE;
		buff->tx_len--;
		
	}	
}

/* Read byte from the receiving buffer. The character will be deleted from the receiving buffer.
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if no character. So, in order to use the function, may be should check the length of the receiving buffer first
 *			in case the buffer can contains '\0';
 *		the first character in the buffer
 */
 
uint8_t uart_rx_buf_read_byte(uart_buffer_type *buff)
{
	uint8_t ch;
	if (buff->rx_len > 0)
	{
		ch = buff->rx_buf[buff->rx_start_index];
		buff->rx_start_index++;
		buff->rx_start_index %= UART_BUFFER_SIZE;
		buff->rx_len--;
		return ch;
	}
	else
		return 0;
}

/* Read number of bytes from the receiving buffer. The characters will be deleted from the receiving buffer.
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @param	*result: a pointer to a place to contain the results
 * @param	number_of_byte: the number of bytes to read
 * @return	the number of character that actually read.
 */
 
uint8_t uart_rx_buf_read(uart_buffer_type *buff, uint8_t *result, uint8_t number_of_byte)
{
	uint8_t i;
	for (i = 0; i < number_of_byte; i++)
	{
		if (buff->rx_len > 0)
		{
			*(result + i) = buff->rx_buf[buff->rx_start_index];
			buff->rx_start_index++;
			buff->rx_start_index %= UART_BUFFER_SIZE;
			buff->rx_len--;
			
		}
		else
			break;
	}
	return i;
}


/* Write byte ch to the receiving buffer 
 * @param	ch: byte to write to the buffer
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if sucessful
 *		1 if the buffer is full
 */
 
uint8_t uart_rx_buf_write_byte(uint8_t ch, uart_buffer_type *buff)
{
	uint16_t tmp;
	if (buff->rx_len < UART_BUFFER_SIZE)
	{
		tmp = (buff->rx_start_index + buff->rx_len) % UART_BUFFER_SIZE;
		buff->rx_buf[tmp] = ch;
		buff->rx_len++;
		return 0;
	}

	else
		return 1;

}


/* Write string to the receiving buffer 
 * @param	charBuff[]: string to write to the buffer
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if sucessful
 *		1 if the buffer is full
 */

uint8_t uart_rx_buf_write(char charBuf[], uart_buffer_type *buff)
{
	uint16_t tmp;
	uint16_t i;
	uint8_t result;
	i = 0;
	result = 0;
	while (charBuf[i]!= '\0')
	{
	        if (buff->rx_len < UART_BUFFER_SIZE)
		{
			tmp = (buff->rx_start_index + buff->rx_len) % UART_BUFFER_SIZE;
			buff->rx_buf[tmp] = charBuf[i];
			buff->rx_len++;
			i++;
		}
		else
		{
			result = 1;
			break;
		}
	}
	return result;
}


/* Write string to the transmitting buffer 
 * @param	charBuff[]: string to write to the buffer
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if sucessful
 *		1 if the buffer is full
 */

uint8_t uart_tx_buf_write(char charBuf[], uart_buffer_type *buff)
{
	uint16_t tmp;
	uint16_t i;
	uint8_t result;
	i = 0;
	result = 0;
	while (charBuf[i]!= '\0')
	{
		if (buff->tx_len < UART_BUFFER_SIZE)
		{
			tmp = (buff->tx_start_index + buff->tx_len) % UART_BUFFER_SIZE;
			buff->tx_buf[tmp] = charBuf[i];
			buff->tx_len++;
			i++;
		}
		else
		{
			result = 1;
			break;
		}
	}
	return result;
}

/* Write byte ch to the transmitting buffer 
 * @param	ch: byte to write to the buffer
 * @param	*buff: a pointer to the data structer that contain the transmitting and receiving buffer
 * @return	0 if sucessful
 *		1 if the buffer is full
 */
 
uint8_t uart_tx_buf_write_byte(uint8_t ch, uart_buffer_type *buff)
{
	uint16_t tmp;
	if (buff->tx_len < UART_BUFFER_SIZE)
	{
		tmp = (buff->tx_start_index + buff->tx_len) % UART_BUFFER_SIZE;
		buff->tx_buf[tmp] = ch;
		buff->tx_len++;
		return 0;
	}
	else
		return 1;
}
