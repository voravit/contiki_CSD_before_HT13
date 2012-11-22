/*
 * adc.c
 *
 *  Created on: 21/02/2012
 *      Author: Jorge Querol
 */

#include "../LPC17xx.h"
#include "../adc/adc_lpc17xx.h"

int ADC[ADCChannels];
int UpdateChannel;

int pos[ADCChannels];
int glitches[ADCChannels];
int Buffer[2][GlitchBuffer][ADCChannels];

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
	NVIC_SetPriority(ADC_IRQn, 2);
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
