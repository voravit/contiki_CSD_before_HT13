/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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

#include <stdlib.h>
#include <stdio.h>

#include "contiki.h"
#include "lib/sensors.h"
#include "dev/pms5003.h"
#include "dev/pms5003-sensor.h"

/*---------------------------------------------------------------------------*/
PROCESS(pmstest, "PMS test");
AUTOSTART_PROCESSES(&pmstest,&sensors_process);
SENSORS(&pms5003_sensor);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pmstest, ev, data)
{
  
  PROCESS_BEGIN();

  pms5003_sensor.configure(SENSORS_ACTIVE, 1);

  while(1) {
    do {
      PROCESS_WAIT_EVENT();
    } while ((ev != sensors_event) && (data != &pms5003_sensor));
    printf("PMS sensor event. PM01 = %04d, PM2.5 = %04d, PM10 = %04d\n",
	   pms5003_sensor.value(PMS5003_SENSOR_PM1_0),
	   pms5003_sensor.value(PMS5003_SENSOR_PM2_5),
	   pms5003_sensor.value(PMS5003_SENSOR_PM10_0));
  }
  PROCESS_END();
}
