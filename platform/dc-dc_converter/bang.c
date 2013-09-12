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
 * From the original bang.c file
 * Author: Jorge Queroal
 * Created on: 2011-12-14
 *
 * Ported to contiki by: Voravit Tanyingyong
 *
 */

#include "lpc17xx.h"
#include "bang.h"

state_t CurrentState;

void GPIOInit(void)
{
	LPC_PINCON->PINSEL4 &= ~0x0000000FF;		// Set GPIO control as digital outputs
	LPC_GPIO2->FIODIRL |= 0x000F;			// Set P2.0, P2.1, P2.2, P2.3 as outputs
	LPC_GPIO2->FIOPINL &= ~0x000F;			// Set OFF all Buck and Boost switches
	CurrentState = ALL_OFF;
}

void TimerInit(void)
{
	LPC_SC->PCONP |= (1 << 16);				// Set up Power for RIT
	LPC_SC->PCLKSEL1 |= (1 << 26);			// Peripheral Clock Enabled
	LPC_RIT->RICTRL &= ~0x0C;				// Disable Repetitive Interrupt Timer (RIT)
	LPC_RIT->RICTRL |= 0x01;				// Clear Repetitive Interrupt Timer (RIT)
	NVIC_EnableIRQ(RIT_IRQn);
	NVIC_SetPriority(RIT_IRQn, 0);
}

void RIT_IRQHandler(void)
{
	LPC_RIT->RICTRL &= ~0x08;					// Disable Repetitive Interrupt Timer (RIT)

	// Shoot-through & Reserve current protection
	if(CurrentState == BUCK_OFF)				// Buck L ON Buck H OFF - Boost L OFF Boost H ON
	{
		LPC_GPIO2->FIOSETL = 0x0009;
		LPC_RIT->RICTRL |= 0x09;						// Enable and clear interrupt
		LPC_RIT->RICOMPVAL = LPC_RIT->RICOUNTER + 700;	// 8us delay (100 cycles +)
		CurrentState = BUCK_SOFT;
	}
	else if(CurrentState == BUCK_SOFT)			// Buck L OFF Buck H OFF - Boost L OFF Boost H ON
	{
		LPC_GPIO2->FIOCLRL = 0x0001;
		LPC_GPIO2->FIOSETL = 0x0008;
	}
	else if(CurrentState == BUCK_ON)			// Buck L OFF Buck H ON - Boost L OFF Boost H ON
		LPC_GPIO2->FIOSETL = 0x000A;
	else if(CurrentState == BOOST_OFF)			// Buck L OFF Buck H ON - Boost L OFF Boost H ON
	{
		LPC_GPIO2->FIOSETL = 0x000A;
		LPC_RIT->RICTRL |= 0x09;						// Enable and clear interrupt
		LPC_RIT->RICOMPVAL = LPC_RIT->RICOUNTER + 700;	// 8us delay (100 cycles +)
		CurrentState = BOOST_SOFT;
	}
	else if(CurrentState == BOOST_SOFT)			// Buck L OFF Buck H ON - Boost L OFF Boost H OFF
	{
		LPC_GPIO2->FIOCLRL = 0x0008;
		LPC_GPIO2->FIOSETL = 0x0002;
	}
	else if(CurrentState == BOOST_ON)			// Buck L OFF Buck H ON - Boost L ON Boost H OFF
		LPC_GPIO2->FIOSETL = 0x0006;
	else if(CurrentState == DISCHARGE)			// Buck L ON Buck H OFF - Boost L ON Boost H OFF
		LPC_GPIO2->FIOSETL = 0x0005;
}

void SetState(state_t state)
{
	if(state == BUCK_OFF)								// Buck L ON Buck H OFF - Boost L OFF Boost H ON
	{
		if(LPC_GPIO2->FIOPINL & 0x0002)
			LPC_GPIO2->FIOCLRL = 0x0002;
		else
			LPC_GPIO2->FIOSETL = 0x0001;

		if(LPC_GPIO2->FIOPINL & 0x0004)
			LPC_GPIO2->FIOCLRL = 0x0004;
		else
			LPC_GPIO2->FIOSETL = 0x0008;
	}
	else if(state == BUCK_SOFT)							// Buck L OFF Buck H OFF - Boost L OFF Boost H ON
	{
		if(LPC_GPIO2->FIOPINL & 0x0001)
			LPC_GPIO2->FIOCLRL = 0x0001;

		if(LPC_GPIO2->FIOPINL & 0x0002)
			LPC_GPIO2->FIOCLRL = 0x0002;

		if(LPC_GPIO2->FIOPINL & 0x0004)
			LPC_GPIO2->FIOCLRL = 0x0004;
		else
			LPC_GPIO2->FIOSETL = 0x0008;
	}
	else if((state == BUCK_ON)||(state == BOOST_OFF))	// Buck L OFF Buck H ON - Boost L OFF Boost H ON
	{
		if(LPC_GPIO2->FIOPINL & 0x0001)
			LPC_GPIO2->FIOCLRL = 0x0001;
		else
			LPC_GPIO2->FIOSETL = 0x0002;

		if(LPC_GPIO2->FIOPINL & 0x00004)
			LPC_GPIO2->FIOCLRL = 0x0004;
		else
			LPC_GPIO2->FIOSETL = 0x0008;
	}
	else if(state == BOOST_SOFT)						// Buck L OFF Buck H ON - Boost L OFF Boost H OFF
	{
		if(LPC_GPIO2->FIOPINL & 0x0001)
			LPC_GPIO2->FIOCLRL = 0x0001;
		else
			LPC_GPIO2->FIOSETL = 0x0002;

		if(LPC_GPIO2->FIOPINL & 0x0004)
			LPC_GPIO2->FIOCLRL = 0x0004;

		if(LPC_GPIO2->FIOPINL & 0x0008)
			LPC_GPIO2->FIOCLRL = 0x0008;
	}
	else if(state == BOOST_ON)							// Buck L OFF Buck H ON - Boost L ON Boost H OFF
	{
		if(LPC_GPIO2->FIOPINL & 0x0001)
			LPC_GPIO2->FIOCLRL = 0x0001;
		else
			LPC_GPIO2->FIOSETL = 0x0002;

		if(LPC_GPIO2->FIOPINL & 0x0008)
			LPC_GPIO2->FIOCLRL = 0x0008;
		else
			LPC_GPIO2->FIOSETL = 0x0004;
	}
	else if(state == DISCHARGE)							// Buck L ON Buck H OFF - Boost L ON Boost H OFF
	{
		if(LPC_GPIO2->FIOPINL & 0x0002)
			LPC_GPIO2->FIOCLRL = 0x0002;
		else
			LPC_GPIO2->FIOSETL = 0x0001;

		if(LPC_GPIO2->FIOPINL & 0x0008)
			LPC_GPIO2->FIOCLRL = 0x0008;
		else
			LPC_GPIO2->FIOSETL = 0x0004;
	}
	else if(state == ALL_OFF)							// Buck L OFF Buck H OFF - Boost L OFF Boost H OFF
		LPC_GPIO2->FIOCLRL = 0x000F;

	if(state != CurrentState)
	{
		LPC_RIT->RICTRL |= 0x09;						// Enable and clear interrupt
		LPC_RIT->RICOMPVAL = LPC_RIT->RICOUNTER + 100;	// 2us delay (100 cycles +)
		CurrentState = state;
	}
}
