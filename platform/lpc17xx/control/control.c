/*
 * control.cimax 6
 *
 *
 *  Created on: 08/03/2012
 *      Author: Jorge Querol
 */

#include "../LPC17xx.h"
#include "../bang/bang.h"
#include "../control/control.h"

#ifdef ShowPeriodically
#include "../comm/comm.h"
#endif

int Vref, Imax, Vmax;
int Vout, Vin, Il;
int Vo, Vi, Io, Ii;
int i, Vom, Vim, Iom, Iim;

extern int ADC[4];
extern state_t CurrentState;

void ValueInit(void)
{
	Vref = 0;
	Imax = (int)(I_IMAX * 4095 * 0.151) /(Vdd * 2);
	Vmax = (int)(I_VMAX * 4095 * 3) /(Vdd * 28);
	Vo = 0;
	Vi = 0;
	Io = 0;
	Ii = 0;
	i = 1;
	Vom = 0;
	Vim = 0;
	Iom = 0;
	Iim = 0;
}

void BangBang(void)
{
	if(Vref > 0)
	{
		/* Safe maximum voltage limitation (it uses configuration number4) */
		if (Vout > Vmax)
			SetState(DISCHARGE);
		/* Current limitation */
		else if(Il > Imax)
		{
			/* If output is over input it's possible to limit the current with state 2 */
			if(Vout > Vin)
				SetState(BOOST_OFF);
			/* else it must be limited with state 1 */
			else
				SetState(BUCK_OFF);
		}
		else
		{
			/* To make the decision of which converter must be used, it's necessary to check
			 * the relationship between Vin and Vref but it's also necessary to know if the
			 * buck converter is in the low conversion efficiency zone. Check operating zone
			 * function in software. */

			/* Boost single converter */
//			if(operatingzone(Vin, Vref, Vout))
			if(Vref > Vin)
			{
				/* OFF mode (state 2 for boost converter) */
				if (Vout > Vref)
				{
					if(Il > 10)
						SetState(BOOST_OFF);
					else
						SetState(BOOST_SOFT);
				}
				/* ON mode (state 3 for boost converter) */
				else
					SetState(BOOST_ON);
			}
			/* Buck single converter */
			else
			{
				/* OFF mode (state 1 for buck converter) */
				if (Vout > Vref)
				{
					if(Il > 10)
						SetState(BUCK_OFF);
					else
						SetState(BUCK_SOFT);
				}
				/* ON mode (state 2 for buck converter) */
				else
					SetState(BUCK_ON);
			}
		}
	}
	else
		SetState(ALL_OFF);
}

void MeanValues(void)
{
	float aux;

	i++;
	Vom += Vout;
	Vim += Vin;
	if((LPC_GPIO2->FIOPIN & 0x0008) || (CurrentState == BOOST_SOFT))
		Iom += Il;
	if(LPC_GPIO2->FIOPIN & 0x0002)
		Iim += Il;

	if((i % 100000) == 0)
	{
		Vo = Vom / 100000;
		Vi = Vim / 100000;
		Io = Iom / 100000;
		Ii = Iim / 100000;
		Vom = 0;
		Vim = 0;
		Iom = 0;
		Iim = 0;
}
#ifdef ShowPeriodically
	if(i >= 100000)
	{
		i = 1;
		print("\r\n Vo:");
		aux = (Vo * Vdd * 28) / (4095 * 3);
		print_float(&aux);
		print(" Vin:");
		aux = (Vi * Vdd * 28) / (4095 * 3);
		print_float(&aux);
		print(" Io:");
		aux = (Io * Vdd * 2) / (4095 * 0.151);
		print_float(&aux);
		print(" Ii:");
		aux = (Ii * Vdd * 2) / (4095 * 0.151);
		print_float(&aux);
	}
#endif
}
