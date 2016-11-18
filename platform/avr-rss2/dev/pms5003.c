/*
 * Copyright (c) 2015, Copyright Robert Olsson / Radio Sensors AB  
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
 *
 * Author  : Robert Olsson robert@radio-sensors.com
 * Created : 2015-11-22
 */

/**
 * \file
 *         A simple application showing sensor reading on RSS2 mote
 */

#include "contiki.h"
#include "sys/etimer.h"
#include <stdio.h>
#include "adc.h"
#include "i2c.h"
#include "dev/leds.h"
#include "dev/battery-sensor.h"
#include "dev/temp-sensor.h"
#include "dev/temp_mcu-sensor.h"
#include "dev/light-sensor.h"
#include "dev/pulse-sensor.h"
#ifdef CO2
#include "dev/co2_sa_kxx-sensor.h"
#endif
#include "dev/rs232.h"
#include "dev/serial-line.h"
#include "dev/serial-raw.h"

#define PMSFRAMELEN 28
#define PMSBUFFER PMSFRAMELEN+2

/* Two Preamble bytes */
#define PRE1 0x42
#define PRE2 0x4d

process_event_t pms5003_event;
static uint16_t PM1, PM2_5, PM10;
static uint32_t invalid_frames, valid_frames;
/*---------------------------------------------------------------------------*/
PROCESS(pms5003_process, "PMS5003 dust sensor process");
 /* AUTOSTART_PROCESSES(&start_process, &pms5003_process);*/
/*---------------------------------------------------------------------------*/
void pms5003_init() {
  rs232_set_input(RS232_PORT_0, serial_raw_input_byte);
  serial_raw_init();
  process_start(&pms5003_process, NULL);
  return;
}

void pms5003_off() {
  return;
}

uint16_t pms5003_pm1() {
  return PM1;
}

uint16_t pms5003_pm2_5() {
  return PM2_5;
}

uint16_t pms5003_pm10() {
  return PM10;
}

uint32_t pms5003_valid_frames() {
  return valid_frames;
}

uint32_t pms5003_invalid_frames() {
  return invalid_frames;
}

/*---------------------------------------------------------------------------*/

/* Validate frame by checking length field and checksum */
static int check_pmsframe(uint8_t *buf) {
  int sum, pmssum;
  int i;
  int len = (buf[0] << 8) + buf[1];
  int valid = 1;
  
  if (len != PMSFRAMELEN) {
    valid = 0;
    goto done;
  }
  /* Compute checksum */
  sum = PRE1+PRE2; /* Fixed preamble */
  /* Exclude checksum field */
  for (i = 0; i < PMSBUFFER-2; i++) {
    sum += buf[i];
  }
  /* Compare with received checksum */
  pmssum = (buf[PMSBUFFER-2] << 8) + buf[PMSBUFFER-1];
  valid = (pmssum == sum);
done:
  if (valid) {
    valid_frames++;
    return 1;
  } else {
    invalid_frames++;
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
extern process_event_t serial_raw_event_message;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pms5003_process, ev, data)
{
  static uint8_t buf[PMSBUFFER];
  static int i;
  
  PROCESS_BEGIN();

  pms5003_event = process_alloc_event();

  while(1) {
    /* First check for two bytes of fixed preamble  */
    do {
      PROCESS_WAIT_EVENT();
      leds_on(LEDS_RED);
    } while (ev != serial_raw_event_message);
 
    if (*((uint8_t *) data) != 0x42)
      continue;
    do {
      PROCESS_WAIT_EVENT();
    } while (ev != serial_raw_event_message);
    if (*((uint8_t *) data) != 0x4d)
      continue;
    /* Found preamble. Get the rest */
    for (i = 0; i < PMSBUFFER; i++) {
      do {
	PROCESS_WAIT_EVENT();
      } while (ev != serial_raw_event_message);
      buf[i] = *((uint8_t *) data);
    }		   
#if 0
    printf("Got a buggef fulle data: ");
    for (i = 0; i < PMSBUFFER; i++) {
	    printf("%02x ", buf[i]);
    }
    printf("\n");
#endif
    /* Verify that is is a valid frame */
    if (check_pmsframe(buf)) {
      /* Update sensor readings */
      PM1 = (buf[2] << 8) | buf[3];
      PM2_5 = (buf[4] << 8) | buf[5];
      PM10 = (buf[6] << 8) | buf[7];      
      printf("valid %lu, invalid %lu: ", valid_frames, invalid_frames);
      printf("PM1 = %04d, PM2.5 = %04d, PM10 = %04d\n", PM1, PM2_5, PM10);
      /* Tell other processes there is new data */
      if (process_post(PROCESS_BROADCAST, pms5003_event, NULL) == PROCESS_ERR_OK) {
	PROCESS_WAIT_EVENT_UNTIL(ev == pms5003_event);
      }
    }
    else
      printf("Frame validation failed\n");
  }
  PROCESS_END();
}

#if 0
PROCESS_THREAD(start_process, ev, data)
{
  PROCESS_BEGIN();
  rs232_set_input(RS232_PORT_0, serial_raw_input_byte);

  serial_raw_init();

  PROCESS_END();
}
#endif
/*---------------------------------------------------------------------------*/
