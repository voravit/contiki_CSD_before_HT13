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
 * From the original adc.c file
 * Author: Jorge Queroal
 * Created on: 2012-02-21
 *
 * Ported to contiki by: Voravit Tanyingyong
 *
 */

#include "contiki.h"
#include "lpc17xx.h"
#include "adc.h"
#include "control.h"
//#include "control.c"

int ADC[ADCChannels];
int UpdateChannel;

int pos[ADCChannels];
int glitches[ADCChannels];
int Buffer[2][GlitchBuffer][ADCChannels];

float get_ADC()
{

}

void set_ADC()
{

}

void ADCvaluesInit(void);

void ADCInit(void)
{
	LPC_PINCON->PINSEL1 |= 0x00154000;		// Select ADC function for pins (ADC0-3)
	LPC_PINCON->PINSEL3 |= 0xF0000000;		// Select ADC function for pins (ADC4-5)
	LPC_PINCON->PINMODE1 |= 0x0002A8000;	// Neither pull-up nor pull-down resistor
	LPC_PINCON->PINMODE3 |= 0xA00000000;	// Neither pull-up nor pull-down resistor
	LPC_SC->PCONP |= (1 << 12);				// Set up bit PCADC
	LPC_SC->PCLKSEL0 |= (01 << 24);			// PCLK = CCLK (102 MHz)
	/* 7-0 SEL
	 * 15-8 CLKDIV = 7 (12.25 MHz)
	 * 15-8 CLKDIV = 3 (25.5 MHz)
	 * 16 BURST software
	 * 21 PDN on (ADC on)
	 */
	LPC_ADC->ADCR |= 0x00200300;
	LPC_ADC->ADINTEN = 0x00000100;			// ADC Interrupt Enabled
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn, 0);
	ADCvaluesInit();						// Init ADC glitch filter
	UpdateChannel = -1;
	ADCRead(0);								// Start first conversion
}

void ADCRead(unsigned char ADC)
{
	LPC_ADC->ADCR &= ~(0x000000FF);			// Remove ADC selected
	LPC_ADC->ADCR |= (1 << ADC);			// Select ADC
	LPC_ADC->ADCR |= (1 << 24);				// Start conversion
}

void ADC_IRQHandler(void)
{
	int Channel;
	Channel = (LPC_ADC->ADGDR >> 24) & 0x00000007;
	ADC[Channel] = (LPC_ADC->ADGDR >> 4) & 0x00000FFF;
	UpdateChannel = Channel;
	Channel++;
	if(Channel < ADCChannels)
		ADCRead(Channel);
	else
	{
		LPC_GPIO1->FIOPIN ^= (1 << 29);
		//ADCRead(0);
		/**/
		process_poll(&bang_control_process);
	}
}

void ADCvaluesInit(void)
{
	int i, j, k;
	for(i=0; i<2; i++)
	{
		for(j=0; j<GlitchBuffer; j++)
		{
			for(k=0; k<ADCChannels; k++)
			{
				Buffer[i][j][k]=0;
			}
		}
	}
	for(i=0; i<ADCChannels; i++)
	{
		pos[i] = 0;
		glitches[i] = 0;
	}
}

int ADCValues(int Channel)
{
#ifdef PREFILTER
	static int i, match;
#endif

	static int sample;
	sample = ADC[Channel];

#ifdef PREFILTER
	match = 0;

	while(((sample - Buffer[0][pos[Channel]][Channel]) > (GlitchHyst))||((Buffer[0][pos[Channel]][Channel] - sample) > (GlitchHyst)))
	{
		pos[Channel]++;
		if(pos[Channel] >= GlitchBuffer)
			pos[Channel] = 0;
		match++;
		if(match >= GlitchBuffer)
			break;
	}

	if(match >= GlitchBuffer)					// Possible glitch detected
	{
		glitches[Channel]++;
		if(glitches[Channel] > GlitchBuffer)	// Previous samples maybe were not glitches
		{
			for(i=0; i<GlitchBuffer; i++)
				Buffer[0][i][Channel] = Buffer[1][i][Channel];
			glitches[Channel] = 0;
		}
		else									// Store suspicious samples
			Buffer[1][glitches[Channel]-1][Channel] = sample;
		sample = 0;
		for(i=0; i<GlitchBuffer; i++)
			sample += Buffer[0][i][Channel];
		sample /= GlitchBuffer;
	}
	else
	{
		glitches[Channel] = 0;
		pos[Channel]--;
		if(pos[Channel] < 0)
			pos[Channel] = GlitchBuffer - 1;
		Buffer[0][pos[Channel]][Channel] = sample;
	}
#endif

	return(sample);
}
