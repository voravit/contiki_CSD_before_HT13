#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"


/* Define which resources to include to meet memory constraints. */
#define REST_RES_HELLO 0
#define REST_RES_LEDS 1
#define REST_RES_TOGGLE 1
//DC-DC converter specific resources
#define REST_RES_SVECTOR 1
#define REST_RES_CTRLPARAM 1
#define REST_RES_DCDCDEBUG 1

#include "erbium.h"
#include "er-coap-07-engine.h"

#if defined (PLATFORM_HAS_ADC)
#include "adc-sensors.h"
#include "bang-control.h"
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

/**
 * Microgrid server IP address //TODO: move to separate file
 */
#define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0x2000, 0, 0, 0, 0, 0, 0, 0x0001)
#define DCDCSERNUM "dcdc1" //TODO: move to separate file
#define LOCAL_PORT      UIP_HTONS(COAP_DEFAULT_PORT+1) //TODO: move to separate file
#define REMOTE_PORT     UIP_HTONS(COAP_DEFAULT_PORT) //TODO: move to separate file

/**
 *	Client beacon period
 */
#define TOGGLE_INTERVAL 5 //TODO: move to separate file

/**
 * Interval for asynchronious CoAP sender to check for messages
 */
#define INTERVAL_ASYNC_COAP_SENDER 2 		//TODO: move to separate file
uip_ipaddr_t server_ipaddr;     				// used to store destination ip address
static struct etimer et;        				// timer struct for client toggling
static struct etimer et2;								// timer for coap sender
char* service_urls[2] = {"mgserver/hello","mgserver/auth"};
static coap_packet_t requestNew[1]; 		// CoAP request packet to be sent
int has_coap_to_send = 0;
static int requiredVoltage;
static int requiredCurrent;
#define DCDC_DEBUG_REQUIREDVOLTAGE  "rV"
#define DCDC_DEBUG_REQUIREDCURRENT "rI"
const char * coapBuff[255]; 						// buffer for CoAP message //TODO: re-use some other buffer
const uint8_t * authResponseOk = "{\"result\":\"ok\"}"; // Possible response to auth request
const uint8_t * authResponseForbidden = "{\"result\":\"forbidden\"}"; // Possible response to auth request

/******************************************************************************/
#if REST_RES_DCDCDEBUG

/**
 * Resource for setting/reading debug parameters
 * Parameters are identified by query parameter "name", value is set as POST parameter "value"
 */
RESOURCE(debugParams, METHOD_GET | METHOD_POST, "debug/parameters", "title=\"DebugParameters: ?len=0..\";rt=\"Text\"");

void
debugParams_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
 int len = 0;
 const char *name = NULL;
 const char *value = NULL;
 int success = 0;
 int coap_method = REST.get_method_type(request);
 PRINTF("DEBUG command method:%d\n", coap_method);

 if ((len=REST.get_query_variable(request, "name", &name))) {
	 if (coap_method == METHOD_POST && REST.get_post_variable(request, "value", &value))
	 {
		 if (strncmp(name, DCDC_DEBUG_REQUIREDVOLTAGE, len)==0)
		 {
			 requiredVoltage = atoi(value);
			 success = 1;
		 } else if (strncmp(name, DCDC_DEBUG_REQUIREDCURRENT, len)==0)
		 {
			 requiredCurrent = atoi(value);
			 success = 1;
		 }
		 } else if (coap_method == METHOD_GET) {
			 if (strncmp(name, DCDC_DEBUG_REQUIREDVOLTAGE, len)==0)
		 {
				 snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "%d", requiredVoltage);
			 success = 1;
		 } else if (strncmp(name, DCDC_DEBUG_REQUIREDCURRENT, len)==0)
		 {
			 snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "%d", requiredCurrent);
			 success = 1;
		 }
			 if (success) {
				 REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
				 REST.set_response_payload(response, buffer, strlen((char *)buffer));
			 }
	 }

 } else {
	 PRINTF("Parameter name not provided");
	 snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "Parameter name not provided");
	 REST.set_response_payload(response, buffer, strlen((char *)buffer));
	 REST.set_response_status(response, REST.status.BAD_REQUEST);
 }

 if (!success) {
	 REST.set_response_status(response, REST.status.BAD_REQUEST);
 }
}

/******************************************************************************/
// Instruct board to send authorize message to the server
// POST parameters : "rV" - requested voltage, "rI" - requested current
RESOURCE(debugauthorize, METHOD_POST, "debug/authorize", "title=\"Hello world: ?len=0..\";rt=\"Text\"");

void
debugauthorize_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
 PRINTF("DEBUG command !!!!, authorize V:%d, I:%d\n", requiredVoltage, requiredCurrent);

 PRINTF("--Creating async CoAP message--\n");

 /* prepare request, TID is set by COAP_BLOCKING_REQUEST() */
 coap_init_message(requestNew, COAP_TYPE_CON, COAP_POST, 0 );
 coap_set_header_uri_path(requestNew, service_urls[1]);

 /* Set Proxy URI*/
 coap_set_header_proxy_uri(requestNew, "http://mgserver/auth");

 /* Set Content type to JSON*/
 coap_set_header_content_type(requestNew, REST.type.APPLICATION_JSON);

 /* Set Payload content as a JSON string*/
 int len = snprintf((char *)coapBuff, REST_MAX_CHUNK_SIZE, "{\"sernum\":\"%s\",\"voltageRequested\":%d,\"currentRequested\":%d}", DCDCSERNUM, requiredVoltage, requiredCurrent);
 coap_set_payload(requestNew, (uint8_t *)coapBuff, strlen((char *)coapBuff));

 // Print coapBuffer contents
 printf("|%.*s", len, (char *)coapBuff);
 has_coap_to_send = 1;
 PRINTF("--Done--\n");

 REST.set_response_status(response, REST.status.OK);
}

#endif

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

/**
 * Handler for power authorization response
 */
void
auth_response_handler(void *response)
{
	uint8_t *chunk;
	coap_get_payload(response, &chunk);
	PRINTF("Received response length: %d\n", len);
	if (strcmp(chunk, authResponseOk) == 0) {
		PRINTF("Received AUTHORIZED\n");
		leds_on(LEDS_GREEN);
		// Turn on power out
	} else {
		PRINTF("Received FORBIDDEN");
		leds_off(LEDS_GREEN);
		// TUrn off power out
	}
}

/**
 * Process for asynchroniously sending CoAP messages. Check variable has_coap_to_send and send CoAP request saved in requestNew
 * WARNING Doesn't have a queue, last CoAP request wil be sent
 */
PROCESS(dcdc_coap_sender, "DCDC coap sender");
PROCESS_THREAD(dcdc_coap_sender, ev, data)
{
  PROCESS_BEGIN();

  SERVER_NODE(&server_ipaddr);

  /* receives all CoAP messages */
  printf("--Initializing CoAP async sender--\n");
  coap_receiver_init();

  etimer_set(&et, INTERVAL_ASYNC_COAP_SENDER * CLOCK_SECOND);

  while(1) {
   PROCESS_YIELD();

   if (etimer_expired(&et)) {
       if (has_coap_to_send) {
         printf("--Sending async CoAP message--\n");
         COAP_BLOCKING_REQUEST(&server_ipaddr, REMOTE_PORT, requestNew, auth_response_handler);
         has_coap_to_send = 0;
         printf("--Done--\n");

       } else {
               printf("--No messages found for sending--\n");
       }
       etimer_reset(&et);
   }

  }

  PROCESS_END();
}

/**
 * Handler for power authorization response
 */
void
beacon_response_handler(void *response)
{
	uint8_t *chunk;
	coap_get_payload(response, &chunk);
	PRINTF("Received response length: %d\n", len);
}

/**
 * DCDC beacon client, periodically sends it's status vector to the microgrid server
 */
PROCESS(dcdc_client, "DCDC beacon client");
PROCESS_THREAD(dcdc_client, ev, data)
{
  PROCESS_BEGIN();

  static coap_packet_t request[1]; /* This way the packet can be treated as pointer as usual. */
  SERVER_NODE(&server_ipaddr);

  etimer_set(&et2, TOGGLE_INTERVAL * CLOCK_SECOND);

  while(1) {
   PROCESS_YIELD();

   if (etimer_expired(&et2)) {
     PRINTF("--Sending beacon to mgserver--\n");

     /* prepare request, TID is set by COAP_BLOCKING_REQUEST() */
     coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0 );
     coap_set_header_uri_path(request, service_urls[0]);

     /* Set Proxy URI*/
     coap_set_header_proxy_uri(request, "http://mgserver/hello");

     /* Set Content type to JSON*/
     coap_set_header_content_type(request, REST.type.APPLICATION_JSON);

     /* Set Payload content as a JSON string*/
     char  * buffer[255];
     float vout_value= dc_converter_get_svector_parameter(SVECTOR_SENSOR_VOUT);
     float iout_value= dc_converter_get_svector_parameter(SVECTOR_SENSOR_IOUT);
     float vin_value= dc_converter_get_svector_parameter(SVECTOR_SENSOR_VIN);
     float iin_value= iout_value;
     char * converter_state_string=dc_converter_get_algorithm_state_string();
		 int len = snprintf((char *) buffer, 255,
						"{\"sernum\":\"%s\", \"voltageOut\":%f,\"currentOut\":%f,\"voltageIn\":%f,\"currentIn\":%f,\"bangState\":\"%s\"}",
						DCDCSERNUM, vout_value, iout_value, vin_value, iin_value, converter_state_string);
     coap_set_payload(request, (uint8_t *)buffer, len);

     COAP_BLOCKING_REQUEST(&server_ipaddr, REMOTE_PORT, request, beacon_response_handler);

     PRINTF("\n--Done--\n");

     etimer_reset(&et2);
   }
  }

  PROCESS_END();
}


PROCESS(rest_server_example, "DCDC CONVERTER COAP CLIENT");
AUTOSTART_PROCESSES(&rest_server_example, &dcdc_coap_sender, &dcdc_client);

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
#if REST_RES_DCDCDEBUG
  rest_activate_resource(&resource_debugauthorize);
  rest_activate_resource(&resource_debugParams);
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
