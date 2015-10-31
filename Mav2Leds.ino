
/*
   Richter, 20151017
   Copied from 
    Program    : mav2hott
    Version    : v0.83, 10.02.2013
    Author     : Menno de Gans (joebar.rc@googlemail.com)

 
*/
//////////////////////////////////////////////////////////////////////////
//
//
//
//  Description: 
// 
//  This Arduino sketch is used for listen to mavlink commands
//  and change LED light patterns according to the mavlink messages
//  containing heartbeat, 
//
//  Please see malv2Leds_notes for... well, notes.
//
//
/* **************************************************************************** */

/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */
#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 

#define MAVLINK10     // Are we listening MAVLink 1.0 or 0.9   (0.9 is obsolete now)
/* Debugging options, if defined, they're on */
#define HEARTBEAT     // HeartBeat signal (comment out for off)
#define SERDB         // Output debug mavlink information to SoftwareSerial @ 19200 baud

/* Frequency for updating leds (and maybe debugs) */
#define FREQ  0.1f  // every 0.1s or 10hz
#define TIM_PRESCALER ((FREQ / (6.4e-5)) -1) /* Prescaler for timer */



/* **********************************************/
/* ***************** INCLUDES *******************/
#include <FastSerial.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <GCS_MAVLink.h>
#include <SoftwareSerial.h>
#include "Mav2Leds.h"            /* Configurations */
#include <digitalWriteFast.h>    /* Direct portmanipulation library to replace digitalWrite. This is faster and smaller in code */


// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif

/* *************************************************/
/* ***************** DEFINITIONS *******************/
#define TELEMETRY_SPEED  57600    /* MAVLink telemetry speed. Use AutoQuad settings */
FastSerialPort0(Serial);           /* Our Uart port for Mavlink*/

#define DPL dbSerial.println 
#define DPN dbSerial.print


#ifdef SERDB
  static uint8_t hRX=hRXpin;           /* software serial port for Debug */
  static uint8_t hTX=hTXpin;
  SoftwareSerial dbSerial(hRX,hTX);    /* (rx port,tx port) */
  #define SDB_SPEED 19200               /* Serial Debug speed */
#endif

int messageCounter;
static bool mavlink_active;

/* **********************************************/
/* ***************** SETUP() *******************/
void setup() 
{
  int counter;                            /* Generic Counter */
  Serial.begin(TELEMETRY_SPEED);          /* Initialize Serial port, speed */
  mavlink_comm_0_port = &Serial;          /* setup mavlink port */

  #ifdef SERDB
    dbSerial.begin(SDB_SPEED);
    DPL("INIT: Debug Serial output ready");
  #endif  

  /* Set pins as outputs for leds */
  for(counter = 0; counter < OutSize;counter++) {
    pinModeFast(Out[counter], OUTPUT);
  }
  
  /* initialize Timer1 for interrupt timer events (timerEvent) */
  cli();              // disable global interrupts
  TCCR1A = 0;         // set entire TCCR1A register to 0
  TCCR1B = 0;         // same for TCCR1B
  OCR1A = TIM_PRESCALER;        // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  TCCR1B |= (1 << CS10);    // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
  sei();                    // enable global interrupts:
}

/* **********  interrupt timer call  ***********/
ISR(TIMER1_COMPA_vect)      /* Compare timer vector */
{
  timerEvent();             /* run everything that is on a 'critical' timer */
}

/* * * * * * * * *  MAIN LOOP * * * * * * * * * * */
void loop() 
{  
  read_mavlink(); 
}
