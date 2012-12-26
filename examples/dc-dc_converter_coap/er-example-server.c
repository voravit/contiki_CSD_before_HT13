/*
 * Copyright (c) 2011, Matthias Kovatsch and other contributors.
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
 */

/**
 * \file
 *      Erbium (Er) REST Engine example (with CoAP-specific code)
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"


/* Define which resources to include to meet memory constraints. */
#define REST_RES_HELLO 0
#define REST_RES_MIRROR 0 /* causes largest code size */
#define REST_RES_CHUNKS 0
#define REST_RES_SEPARATE 0
#define REST_RES_PUSHING 0
#define REST_RES_EVENT 0
#define REST_RES_SUB 0
#define REST_RES_LEDS 0
#define REST_RES_TOGGLE 1
#define REST_RES_LIGHT 0
#define REST_RES_BATTERY 0
#define REST_RES_RADIO 0
//DC-DC converter specific resources
#define REST_RES_SVECTOR 1
#define REST_RES_CTRLPARAM 1

#include "erbium.h"
#include "er-coap-07-engine.h"

#if defined (PLATFORM_HAS_ADC)
#include "adc-sensors.h"
#include "bang-control.h"
#endif
#if defined (PLATFORM_HAS_BUTTON)
#include "dev/button-sensor.h"
#endif
#if defined (PLATFORM_HAS_LEDS)
#include "dev/leds.h"
#endif

#if WITH_COAP == 7
#include "er-coap-07.h"
#endif

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]",(lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3],(lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

#define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0x2000, 0, 0, 0, 0, 0, 0, 0x0001)
#define LOCAL_PORT      UIP_HTONS(COAP_DEFAULT_PORT+1)
#define REMOTE_PORT     UIP_HTONS(COAP_DEFAULT_PORT)
#define TOGGLE_INTERVAL 10      // Client beacon period
uip_ipaddr_t server_ipaddr;     // microgrid server ip address
static struct etimer et;        // timer struct for client toggling
char* service_urls[2] = {"mgserver/hello","mgserver/requestResource"};

/* This function is will be passed to COAP_BLOCKING_REQUEST() to handle responses. */
void
client_chunk_handler(void *response)
{
  uint8_t *chunk;

  int len = coap_get_payload(response, &chunk);
  printf("|%.*s", len, (char *)chunk);
}

PROCESS(dcdc_client, "DCDC client");
PROCESS_THREAD(dcdc_client, ev, data)
{
  PROCESS_BEGIN();

  static coap_packet_t request[1]; /* This way the packet can be treated as pointer as usual. */
  SERVER_NODE(&server_ipaddr);

  /* receives all CoAP messages */
  printf("--Initializing coap receiver--\n");
  coap_receiver_init();

  etimer_set(&et, TOGGLE_INTERVAL * CLOCK_SECOND);

  while(1) {
    PROCESS_YIELD();

    if (etimer_expired(&et)) {
      printf("--Sending beacon to mgserver--\n");

      /* prepare request, TID is set by COAP_BLOCKING_REQUEST() */
      coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0 );
      coap_set_header_uri_path(request, service_urls[0]);

      /* Set Proxy URI*/
      coap_set_header_proxy_uri(request, "http://mgserver/hello");

      /* Set Content type to JSON*/
      coap_set_header_content_type(request, REST.type.APPLICATION_JSON);

      /* Set Payload content as a JSON string*/
      const char msg[] = "{\"sernum\":\"dcdcnode1\"}";
      coap_set_payload(request, (uint8_t *)msg, sizeof(msg)-1);

      PRINT6ADDR(&server_ipaddr);
      PRINTF(" : %u\n", UIP_HTONS(REMOTE_PORT));

      COAP_BLOCKING_REQUEST(&server_ipaddr, REMOTE_PORT, request, client_chunk_handler);

      printf("\n--Done--\n");

      etimer_reset(&et);
    }
  }

  PROCESS_END();
}
/******************************************************************************/
#if REST_RES_HELLO
/*
 * Resources are defined by the RESOURCE macro.
 * Signature: resource name, the RESTful methods it handles, and its URI path (omitting the leading slash).
 */
RESOURCE(helloworld, METHOD_GET, "hello", "title=\"Hello world: ?len=0..\";rt=\"Text\"");

/*
 * A handler function named [resource name]_handler must be implemented for each RESOURCE.
 * A buffer for the response payload is provided through the buffer pointer. Simple resources can ignore
 * preferred_size and offset, but must respect the REST_MAX_CHUNK_SIZE limit for the buffer.
 * If a smaller block size is requested for CoAP, the REST framework automatically splits the data.
 */
void
helloworld_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  const char *len = NULL;
  /* Some data that has the length up to REST_MAX_CHUNK_SIZE. For more, see the chunk resource. */
  char const * const message = "Hello World! ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxy";
  int length = 12; /*           |<-------->| */

  /* The query string can be retrieved by rest_get_query() or parsed for its key-value pairs. */
  if (REST.get_query_variable(request, "len", &len)) {
    length = atoi(len);
    if (length<0) length = 0;
    if (length>REST_MAX_CHUNK_SIZE) length = REST_MAX_CHUNK_SIZE;
    memcpy(buffer, message, length);
  } else {
    memcpy(buffer, message, length);
  }

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN); /* text/plain is the default, hence this option could be omitted. */
  REST.set_header_etag(response, (uint8_t *) &length, 1);
  REST.set_response_payload(response, buffer, length);
}
#endif

/******************************************************************************/
#if defined (PLATFORM_HAS_LEDS)
#if REST_RES_LEDS
/*A simple actuator example, depending on the color query parameter and post variable mode, corresponding led is activated or deactivated*/
RESOURCE(leds, METHOD_POST | METHOD_PUT , "actuators/leds", "title=\"LEDs: ?color=r|g|b, POST/PUT mode=on|off\";rt=\"Control\"");

void
leds_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  size_t len = 0;
  const char *color = NULL;
  const char *mode = NULL;
  uint8_t led = 0;
  int success = 1;

  if ((len=REST.get_query_variable(request, "color", &color))) {
    PRINTF("color %.*s\n", len, color);

    if (strncmp(color, "r", len)==0) {
      led = LEDS_RED;
    } else if(strncmp(color,"g", len)==0) {
      led = LEDS_GREEN;
    } else if (strncmp(color,"b", len)==0) {
      led = LEDS_BLUE;
    } else {
      success = 0;
    }
  } else {
    success = 0;
  }

  if (success && (len=REST.get_post_variable(request, "mode", &mode))) {
    PRINTF("mode %s\n", mode);

    if (strncmp(mode, "on", len)==0) {
      leds_on(led);
    } else if (strncmp(mode, "off", len)==0) {
      leds_off(led);
    } else {
      success = 0;
    }
  } else {
    success = 0;
  }

  if (!success) {
    REST.set_response_status(response, REST.status.BAD_REQUEST);
  }
}
#endif

/******************************************************************************/
#if REST_RES_TOGGLE
/* A simple actuator example. Toggles the red led */
RESOURCE(toggle, METHOD_GET | METHOD_PUT | METHOD_POST, "actuators/toggle", "title=\"Red LED\";rt=\"Control\"");
void
toggle_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  leds_toggle(LEDS_ALL);
}
#endif
#endif /* PLATFORM_HAS_LEDS */

/******************************************************************************/
#if defined (PLATFORM_HAS_ADC)

#if REST_RES_SVECTOR
//State vector of the DC-DC converter
RESOURCE(svector, METHOD_GET, "dc-dc/stateVector", "title=\"State vector of the DC-DC converter(Vout, Iout, Vin, In, bangAlgorithmState), supports JSON\";rt=\"StateVector\"");
void
svector_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  float vout_value= dc_converter_get_svector_parameter(SVECTOR_SENSOR_VOUT);
  float iout_value= dc_converter_get_svector_parameter(SVECTOR_SENSOR_IOUT);
  float vin_value= dc_converter_get_svector_parameter(SVECTOR_SENSOR_VIN);
  float iin_value= iout_value;
  char * converter_state_string=dc_converter_get_algorithm_state_string();

  const uint16_t *accept = NULL;
  int num = REST.get_header_accept(request, &accept);

  if ((num==0) || (num && accept[0]==REST.type.TEXT_PLAIN))
  {
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "State:\t%s\nVout:\t%fV\nIout:\t%fA\nVin:\t%fV\nIin:\t%fA", converter_state_string, vout_value, iout_value, vin_value, iin_value);

    REST.set_response_payload(response, (uint8_t *)buffer, strlen((char *)buffer));
  }
  else if (num && (accept[0]==REST.type.APPLICATION_XML))
  {
    REST.set_header_content_type(response, REST.type.APPLICATION_XML);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "<vout=\"%f\" iout=\"%f\" vin=\"%f\" iin=\"%f\" bangState=\"%s\"/>", vout_value, iout_value, vin_value, iin_value, converter_state_string);

    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  }
  else if (num && (accept[0]==REST.type.APPLICATION_JSON))
  {
    REST.set_header_content_type(response, REST.type.APPLICATION_JSON);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "{'svector':{'vout':%f, 'iout':%f, 'vin':%f, 'iin':%f, 'bangState':'%s'}}", vout_value, iout_value, vin_value, iin_value, converter_state_string);
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  }
  else
  {
    REST.set_response_status(response, REST.status.UNSUPPORTED_MADIA_TYPE);
    const char *msg = "Supporting content-types text/plain, application/xml, and application/json";
    REST.set_response_payload(response, msg, strlen(msg));
  }
}


#endif /* REST_RES_SVECTOR */

#if REST_RES_CTRLPARAM
// DCDC converter control parameters
RESOURCE(ctrlparam, METHOD_GET | METHOD_POST, "dc-dc/controlParameters", "title=\"DCDC control parameters\";rt=\"Control\"");
void
ctrlparam_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  const char *variable = NULL;
  int coap_method = coap_get_rest_method(request);
  PRINTF("Received dc-dc/ctrlParameters request:%d\n", coap_method);
  if (coap_method == METHOD_POST)
  {
      if (REST.get_post_variable(request, "userAllowed", &variable) > 0)
      {
          PRINTF("Received POST request for userAllowed\n");
          //printf("Received the following for userAllowed: %s\n", variable);
          char str_yes1[]="YES";
          char str_yes2[]="yes";
          if(strncmp(variable,str_yes1,sizeof(str_yes1)-1) && strncmp(variable,str_yes2,sizeof(str_yes2)-1)){
              dc_converter_forbid_user();
              //printf("Kicking out user\n");
          }
          else{
              dc_converter_allow_user();
              //printf("Allowing user\n");
          }
      }

      if (REST.get_post_variable(request, "Vref", &variable) > 0)
      {
          float v_ref=atoff(variable);
          PRINTF("Received POST request for Vref, new value will be set to %f\n", v_ref);
          //printf("CoAP setting Vref to %f\n", vRef);
          dc_converter_set_control_parameter(CONV_VREF, v_ref);
          //printf("The new value of Vref is %f\n", getConverterParameter(CONV_VREF));
      }
      if (REST.get_post_variable(request, "Vmax", &variable) > 0)
      {
          float v_max=atoff(variable);
          PRINTF("Received POST request for Vmax, new value will be set to %f\n", v_max);
          //printf("CoAP setting Vref to %f\n", vMax);
          dc_converter_set_control_parameter(CONV_VMAX, v_max);
          //printf("The new value of Vmax is %f\n", getConverterParameter(CONV_VMAX));
      }
      if (REST.get_post_variable(request, "Imax", &variable) > 0)
      {
          float i_max=atoff(variable);
          PRINTF("Received POST request for Imax, new value will be set to %f\n", vRef);
          //printf("CoAP setting Imax to %f\n", iMax);
          dc_converter_set_control_parameter(CONV_IMAX, i_max);
          //printf("The new value of Imax is %f\n", getConverterParameter(CONV_IMAX));
      }
  }
  else
  {
     PRINTF("Received GET request for the control parameters of the DC-DC converter\n");
     const uint16_t *accept = NULL;
     int num = REST.get_header_accept(request, &accept);

     char user_allowed_string[4];
      if(dc_converter_get_user_status()){
          strcpy(user_allowed_string, "yes");
      }
      else{
          strcpy(user_allowed_string, "no");
      }
      float v_ref=dc_converter_get_control_parameter(CONV_VREF);
      float v_max=dc_converter_get_control_parameter(CONV_VMAX);
      float i_max=dc_converter_get_control_parameter(CONV_IMAX);

     if ((num==0) || (num && accept[0]==REST.type.TEXT_PLAIN))
     {
         PRINTF("Sending CoAP Text/Plain response\n");
         REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
         snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "userAllowed:\t%s\nVref:\t\t%fV\nVmax:\t\t%fV\nImax:\t\t%fA", user_allowed_string, v_ref, v_max, i_max);
         REST.set_response_payload(response, (uint8_t *)buffer, strlen((char *)buffer));
     }
     else if (num && (accept[0]==REST.type.APPLICATION_JSON))
     {
          PRINTF("Sending JSON response\n");
          REST.set_header_content_type(response, REST.type.APPLICATION_JSON);
          snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "{ctrlParam':{'userAllowed':%s, 'vref':%f, 'vmax':%f, 'imax':%f}}", user_allowed_string, v_ref, v_max, i_max);
          REST.set_response_payload(response, buffer, strlen((char *)buffer));
     }
     else
     {
         PRINTF("Request received, but the server doesn't accept anything other than JSON or TEXT/PLAIN!\n");
     }
  }
}

#endif  /* REST_RES_CTRLPARAM */

#endif /* PLATFOR_HAS_ADC */


PROCESS(rest_server_example, "DCDC CONVERTER COAP CLIENT");
AUTOSTART_PROCESSES(&rest_server_example, &dcdc_client);

PROCESS_THREAD(rest_server_example, ev, data)
{
  PROCESS_BEGIN();

  PRINTF("Starting DCD converter server \n");

  PRINTF("uIP buffer: %u\n", UIP_BUFSIZE);
  PRINTF("LL header: %u\n", UIP_LLH_LEN);
  PRINTF("IP+UDP header: %u\n", UIP_IPUDPH_LEN);
  PRINTF("REST max chunk: %u\n", REST_MAX_CHUNK_SIZE);

  /* Initialize the REST engine. */
  rest_init_engine();

  /* Activate the application-specific resources. */
#if REST_RES_HELLO
  rest_activate_resource(&resource_helloworld);
#endif
#if defined (PLATFORM_HAS_LEDS)
#if REST_RES_LEDS
  rest_activate_resource(&resource_leds);
#endif
#if REST_RES_TOGGLE
  rest_activate_resource(&resource_toggle);
#endif
#endif /* PLATFORM_HAS_LEDS */
#if defined (PLATFORM_HAS_ADC) && REST_RES_SVECTOR
  rest_activate_resource(&resource_svector);
#endif
#if defined (PLATFORM_HAS_ADC) && REST_RES_CTRLPARAM
  rest_activate_resource(&resource_ctrlparam);
#endif

  /* Define application-specific events here. */
  while(1) {
    PROCESS_WAIT_EVENT();
#if defined (PLATFORM_HAS_BUTTON)
    if (ev == sensors_event && data == &button_sensor) {
      PRINTF("BUTTON\n");
#if REST_RES_EVENT
      /* Call the event_handler for this application-specific event. */
      event_event_handler(&resource_event);
#endif
#if REST_RES_SEPARATE && WITH_COAP>3
      /* Also call the separate response example handler. */
      separate_finalize_handler();
#endif
    }
#endif /* PLATFORM_HAS_BUTTON */
  } /* while (1) */

  PROCESS_END();
}
