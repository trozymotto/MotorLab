#ifndef __LEDs_H
#define __LEDs_H

#include <avr/io.h>         //gives us names for registers
#include <avr/interrupt.h>

#include <inttypes.h> //gives us uintX_t

#include <string.h>

/* 
	IO Port Information starts on pg 72 of the Atmega datasheet.
	DDxn : data direction
	PORTxn : drive pin hi or lo (when configured as output)
	PINxn : receive data here (when configured as input)

	LEDS should be on the following pins:
		Yellow : PA0
		Green  : PD5 OC1A
		Red    : PA2

*/

// define the data direction registers
#define DD_REG_RED	   DDRA
#define DD_REG_YELLOW  DDRA
#define DD_REG_GREEN   DDRD

// define the output ports by which you send signals to the LEDs
#define PORT_RED     PORTA
#define PORT_YELLOW  PORTA
#define PORT_GREEN   PORTD

// define the bit-masks for each port that the LEDs are attached to
#define BIT_RED      ( 1 << 2 )
#define BIT_YELLOW   ( 1 << 0 )
#define BIT_GREEN    ( 1 << 5 )

// define "function" calls for turning LEDs on and off
#define LED_ON(x)     (PORT_##x |= BIT_##x)
#define LED_OFF(x)    (PORT_##x &= ~(BIT_##x))
#define LED_TOGGLE(x) (PORT_##x ^= BIT_##x)

// function call prototypes
void init_LEDs();
void set_toggle(char color, int ms);

#endif

