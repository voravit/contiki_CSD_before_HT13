/*
 * config.h
 *
 *  Created on: 23/08/2011
 *      Author: Camilo J. Chaves. camilojchaves@gmail.com
 */

#ifndef CONFIG_H_
#define CONFIG_H_


//UART stuff
	#define UART_CHANNEL 0 //0 or 1 (0 overlaps ADC 6 and 7, but 0 is used for flashing with ISP)


//ADC stuff
	#define ADC_CLK 1000000
	#define MAXTRIESFORSAMPLE 100 //max number of tries to get a reliable sample
	#define ADCS_USED 4 //LPC1768 has 8 adc
	#define ADC_PWR_SRC 0
	#define ADC_CAP_BANK 1
	#define ADC_PSU_OUT 2
	#define ADC_STEPS_AROUND 20 //WHEN LOOKING FOR ReLIABLE DATA, find values into +/- this range
	#define ADC_DEFAULT_RELIABILITY 3 //get X number of similar samples to get a reliable one.

	//multipling factors
	//LPC1768 just measures up to 3.3v, so we have to use a voltage divider
	//to reduce the voltage (100M ohm and 10M ohm will divide 1/10)
	#define ADC_DEFAULT_MULTIPLIER 10
	#define ADC0_MULTIPLIER ADC_DEFAULT_MULTIPLIER
	#define ADC1_MULTIPLIER 1
	#define ADC2_MULTIPLIER 1
	#define ADC3_MULTIPLIER 1
	#define ADC4_MULTIPLIER ADC_DEFAULT_MULTIPLIER
	#define ADC5_MULTIPLIER ADC_DEFAULT_MULTIPLIER
	#define ADC6_MULTIPLIER ADC_DEFAULT_MULTIPLIER
	#define ADC7_MULTIPLIER ADC_DEFAULT_MULTIPLIER

	#define ADC_MAX 0xFFF //The maximun value in the ADC input
	#define VREFP 3.3 	//Has to be VCC=3.3
	#define VREFN 0		//Ground voltage
	#define ADC_CORRECTION 0.952 // it will multiply the measured voltage

	//for the ADC graph and table
	#define ADC_NBR_SAMPLES		100 //# of samples to make the ADC graph
	#define ADC_NBR_OCCUR		20  //values around the average measured value
	//#define ADC_CLK				12000000		/* Just less than 13MHz recommended in datasheet */
	#define EXPORT_EXCEL 1  //Leave this 1

//PWM stuff

#endif /* CONFIG_H_ */
