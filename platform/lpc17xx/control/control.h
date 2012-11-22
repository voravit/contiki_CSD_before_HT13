/*
 * control.h
 *
 *  Created on: 08/03/2012
 *      Author: Jorge Querol
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#define Vdd 3.293
#define I_IMAX	1
#define I_VMAX	25

//#define ShowPeriodically

void ValueInit(void);
void BangBang(void);
void MeanValues(void);

#endif /* CONTROL_H_ */
