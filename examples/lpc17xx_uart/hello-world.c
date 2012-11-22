/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
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
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"

#include <stdio.h> /* For printf() */
#include "LPC17xx.h"
#include "comm.h"


volatile uint32_t temp;

void _delay(uint32_t del);
void set_LEDS();
void set_HIGH(uint32_t led);
void set_LOW(uint32_t led);
int i;

void _delay(uint32_t del)
{
	uint32_t i;
	for(i=0;i<del;i++)
		temp = i;
}

void set_LEDS()
{
	LPC_SC->PCONP |= ( 1 << 15 ); // power up GPIO
  	LPC_GPIO1->FIODIR |= 1 << 29; // puts P1.29 into output
  	LPC_GPIO1->FIODIR |= 1 << 18; // puts P1.18 into output
}

void set_HIGH(uint32_t led)
{
	LPC_GPIO1->FIOPIN |= 1 << led; // make P1.led high
}

void set_LOW(uint32_t led)
{
	LPC_GPIO1->FIOPIN &= ~( 1 << led ); // make P1.led low
}

void blink_N(int n,int delay_ms)
{
 for(i=0;i<n;i++)
 {
    set_HIGH(29); // make P1.18 low
    delayMs(0, delay_ms);
    set_LOW(29); // make P1.29 high
    delayMs(0, delay_ms);
 }
}


/*
* In the system_lpc17xx.c file PCLKSEL0 have been set as 0x00000000 i.e.
* peripheral clock selection for timer 0 will be CCLK/4
* 72 / 4= 18MHz
* 
* 
* 
*/
void delayMs(uint8_t timer_num, uint32_t delayInMs)
{
  if ( timer_num == 0 )/* timer 0 */
  {
        LPC_TIM0->TCR = 0x02;                /* reset timer */
        LPC_TIM0->PR  = 0x00;                /* set prescaler to zero */
        LPC_TIM0->MR0 = delayInMs * (24000000/1000-1);//(9000000 / 1000-1);
        LPC_TIM0->IR  = 0xff;                /* reset all interrrupts */
        LPC_TIM0->MCR = 0x04;                /* stop timer on match */
        LPC_TIM0->TCR = 0x01;                /* start timer */

        /* wait until delay time has elapsed */
        while (LPC_TIM0->TCR & 0x01);
  }
  else if ( timer_num == 1 )/* timer 1 */
  {
        LPC_TIM1->TCR = 0x02;                /* reset timer */
        LPC_TIM1->PR  = 0x00;                /* set prescaler to zero */
        LPC_TIM1->MR0 = delayInMs * (24000000/1000-1);//(9000000 / 1000-1);
        LPC_TIM1->IR  = 0xff;                /* reset all interrrupts */
        LPC_TIM1->MCR = 0x04;                /* stop timer on match */
        LPC_TIM1->TCR = 0x01;                /* start timer */

        /* wait until delay time has elapsed */
        while (LPC_TIM1->TCR & 0x01);
  }
  return;
}


/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();

  set_LEDS();

  //blink_N(3,2000);

  
  while (1) {
    print("Hello World");
    set_HIGH(29); // make P1.18 low
    delayMs(0, 300);
    set_LOW(29); // make P1.29 high
    delayMs(0, 300);
    //printf("Hello, world\n");
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/



