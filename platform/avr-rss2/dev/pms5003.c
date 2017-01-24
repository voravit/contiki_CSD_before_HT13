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

/* Valid values for body length field */
#define PMSMINBODYLEN 20
#define PMSMAXBODYLEN 28
/* Buffer holds frame body plus preamble (two bytes)
 * and length field (two bytes) */
#define PMSBUFFER (PMSMAXBODYLEN+4)

/* Two preamble bytes */
#define PRE1 0x42
#define PRE2 0x4d

process_event_t pms5003_event;
static uint16_t PM1, PM2_5, PM10;
static uint16_t PM1_ATM, PM2_5_ATM, PM10_ATM;
static uint32_t invalid_frames, valid_frames;
/*---------------------------------------------------------------------------*/
PROCESS(pms5003_uart_process, "PMS5003/UART dust sensor process");
PROCESS(pms5003_i2c_process, "PMS5003/I2C dust sensor process");
/*---------------------------------------------------------------------------*/
void pms5003_init() {
  rs232_set_input(RS232_PORT_0, serial_raw_input_byte);
  serial_raw_init();
  pms5003_event = process_alloc_event();
  process_start(&pms5003_uart_process, NULL);
  process_start(&pms5003_i2c_process, NULL);
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

uint16_t pms5003_pm1_atm() {
  return PM1_ATM;
}

uint16_t pms5003_pm2_5_atm() {
  return PM2_5_ATM;
}

uint16_t pms5003_pm10_atm() {
  return PM10_ATM;
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
  int len;

  if (buf[0] != PRE1 || buf[1] != PRE2)
    return 0;
  /* len is length of frame not including preamble and checksum */
  len = (buf[2] << 8) + buf[3];
  if (len < PMSMINBODYLEN || len > PMSMAXBODYLEN) {
    return 0;
  }
  /* Sum data bytewise, including preamble but excluding checksum */
  sum = 0;
  for (i = 0; i < len+2; i++) {
    sum += buf[i];
  }
  /* Compare with received checksum last in frame*/
  pmssum = (buf[len+2] << 8) + buf[len+3];
  return (pmssum == sum);
}
/*---------------------------------------------------------------------------*/
/* Frame received from PMS sensor. Validate and update sensor data. 
 * Return 1 if valid frame, otherwise 0 
 */
static int
pmsframe(uint8_t *buf) {

  if (check_pmsframe(buf)) {
    valid_frames++;
    /* Update sensor readings */
    PM1 = (buf[4] << 8) | buf[5];
    PM2_5 = (buf[6] << 8) | buf[7];
    PM10 = (buf[8] << 8) | buf[9];      
    PM1_ATM = (buf[10] << 8) | buf[11];
    PM2_5_ATM = (buf[12] << 8) | buf[13];
    PM10_ATM = (buf[14] << 8) | buf[15];      
    return 1;
  }
  else {
    invalid_frames++;
    return 0;
  }
}

static void
printpm() {
    printf("PMS frames: valid %lu, invalid %lu\n", valid_frames, invalid_frames);
    printf("PM1 = %04d, PM2.5 = %04d, PM10 = %04d\n", PM1, PM2_5, PM10);
    printf("PM1_ATM = %04d, PM2.5_ATM = %04d, PM10_ATM = %04d\n", PM1_ATM, PM2_5_ATM, PM10_ATM);	
}
  
/*---------------------------------------------------------------------------*/
extern process_event_t serial_raw_event_message;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pms5003_uart_process, ev, data)
{
  static uint8_t buf[PMSBUFFER], *bufp;
  static int remain;

  PROCESS_BEGIN();

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

    /* Found preamble. Then get length */
    bufp = buf;
    do {
      PROCESS_WAIT_EVENT();
    } while (ev != serial_raw_event_message);
    *bufp++ = *((uint8_t *) data);
    do {
      PROCESS_WAIT_EVENT();
    } while (ev != serial_raw_event_message); 
    *bufp++ = *((uint8_t *) data);
    /* Body length in first two bytes -- no of bytes that remain to get */
    remain = (buf[0] << 8) + buf[1];
    if (remain < PMSMINBODYLEN || remain > PMSMAXBODYLEN) {
      invalid_frames++;
    }
    else {
      while (remain--) {
	do {
	  PROCESS_WAIT_EVENT();
	} while (ev != serial_raw_event_message);
	*bufp++ = *((uint8_t *) data);
      }		   
      if (pmsframe(buf)) {
	/* Tell other processes there is new data */
	if (process_post(PROCESS_BROADCAST, pms5003_event, NULL) == PROCESS_ERR_OK) {
	  PROCESS_WAIT_EVENT_UNTIL(ev == pms5003_event);
	}
	
      }
    }
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
static struct etimer i2ctimer;

PROCESS_THREAD(pms5003_i2c_process, ev, data)
{
  static uint8_t buf[PMSBUFFER];
  extern uint16_t i2c_probed; /* contiki_main.c: i2c devices we have probed */

  PROCESS_BEGIN();
  etimer_set(&i2ctimer, CLOCK_SECOND*15);

  /* Main loop */
  while(1) {

    PROCESS_YIELD();

    if((ev == PROCESS_EVENT_TIMER) && (data == &i2ctimer)) {
      if (i2c_probed & I2C_PMS5003) {
	i2c_read_mem(I2C_PMS5003_ADDR, 0, buf, PMSBUFFER);
	if (pmsframe(buf)) {
	  printpm();
	  /* Tell other processes there is new data */
	  if (process_post(PROCESS_BROADCAST, pms5003_event, NULL) == PROCESS_ERR_OK) {
	    PROCESS_WAIT_EVENT_UNTIL(ev == pms5003_event);
	  }
	}
      }
      etimer_reset(&i2ctimer);
    }
  }
  PROCESS_END();
}

