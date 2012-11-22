/*
 * bang.h
 *
 *  Created on: 14/12/2011
 *      Author: Jorge Querol
 */

#ifndef BANG_H_
#define BANG_H_

typedef enum
{
	BUCK_OFF = 0,
	BUCK_SOFT,
	BUCK_ON,
	BOOST_OFF,
	BOOST_SOFT,
	BOOST_ON,
	DISCHARGE,
	ALL_OFF
}state_t;

void GPIOInit(void);
void TimerInit(void);
void SetState(state_t state);

#endif /* __BANG_H__ */
