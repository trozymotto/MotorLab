#include "timer.h"
#include "LEDs.h"
#include "menu.h"

// useful stuff from libc
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>

// GLOBALS
extern uint32_t G_green_ticks;
extern uint32_t G_yellow_ticks;
extern uint32_t G_ms_ticks;

extern uint16_t G_red_period;
extern uint16_t G_green_period;
extern uint16_t G_yellow_period;

extern uint16_t G_release_red;

void init_timers() {

    //int length = 0;
    //char tempBuf[32];
	// -------------------------  RED --------------------------------------//
	// Software Clock Using Timer/Counter 0.
	// THE ISR for this is below.

	// SET appropriate bits in TCCR....

    // Setup timer 0 for a 1ms tick time
    // 20MHz / 
    // 0x80 COM0A1: Clear OC0A on Compare Match
    // 0x02 WGM01: CTC
	// Using CTC mode with OCR0 for TOP. This is mode 2, thus WGM2/1/0 = 010.    
    // Using pre-scaler 256. This is CS0/2/1/0 = 100
    TCCR0A = 0x82;
    TCCR0B = 0x04;
	
	// Software Clock Interrupt Frequency: 1000 = f_IO / (prescaler*OCR0)
	// Set OCR0 appropriately for TOP to generate desired frequency of 1KHz
	// 20000000 / (256 * 78) = 1001.6
	//length = sprintf(tempBuf, "Initializing software clock to freq 1000Hz (period 1 ms)\n");
	//print_usb(tempBuf, length);	
	OCR0A = 78;

	//Enable output compare match interrupt on timer 0A
    TIMSK0 = 0x02;

	// Initialize counter
	G_ms_ticks = 0;

	//--------------------------- YELLOW ----------------------------------//
	// Set-up of interrupt for toggling yellow LEDs. 
	// This task is "self-scheduled" in that it runs inside the ISR that is 
	// generated from a COMPARE MATCH of 
	//      Timer/Counter 3 to OCR3A.
	// Obviously, we could use a single timer to schedule everything, but we are experimenting here!
	// THE ISR for this is in the LEDs.c file

	// SET appropriate bits in TCCR ...
	// Using CTC mode with OCR3A for TOP. This is mode 4, thus WGM1/3210 = 0100.
	// Using pre-scaler 64. This is CS1_210 = 011
    TCCR3A = 0x80;
    TCCR3B = 0x0B;

	// Interrupt Frequency: 10 = f_IO / (prescaler*OCR3A) = 20000000 / (64 * 31250) = 10.00064
	// Set OCR3A appropriately for TOP to generate desired frequency using Y_TIMER_RESOLUTION (100 ms).
	// NOTE: This is not the toggle frequency, rather a tick frequency used to time toggles.
	OCR3A = 31250;
	//printf("Initializing yellow clock to freq %dHz (period %d ms)\n",(int)(10),Y_TIMER_RESOLUTION);	

	//Enable output compare match interrupt on timer 3A
    TIMSK3 |= 0x02;

	G_yellow_ticks = 0;

	//--------------------------- GREEN ----------------------------------//
	// Set-up of interrupt for toggling green LED. 
	// This "task" is implemented in hardware, because the OC1A pin will be toggled when 
	// a COMPARE MATCH is made of 
	//      Timer/Counter 1 to OCR1A.
	// We will keep track of the number of matches (thus toggles) inside the ISR (in the LEDs.c file)
	// Limits are being placed on the frequency because the frequency of the clock
	// used to toggle the LED is limited.

	// Using CTC mode with OCR1A for TOP. This is mode XX, thus WGM3/3210 = .
    // PD5 PWM output
    // Mode 14 wgm, clock select, ocr 1a to half of ocr
    // Toggle OC1A on a compare match. Thus COM1A_10 = 01
    // Using pre-scaler 1024. This is CS1/2/1/0 = 101
    TCCR1A = 0x40;
    TCCR1B = 0x0D;

	// Interrupt Frequency: ? = f_IO / (1024*OCR1A)
	// equation ==>  OCR1A = period * 20000000 / (1024 * 1000) = period * 19.53
	// Set OCR1A appropriately for TOP to generate desired frequency.
	// NOTE: This IS the toggle frequency.
	//length = sprintf(tempBuf, "green period %d\n",G_green_period);
	//print_usb(tempBuf, length);
 	OCR1A = (uint16_t) (G_green_period * 19.53);
	//length = sprintf(tempBuf, "Set OCR1A to %d\n",OCR1A);
	//print_usb(tempBuf, length);
 	//length = sprintf(tempBuf, "green init freq %dHz (period %d ms)\n",
 	//                      (int)(19531.25/OCR1A),G_green_period);	
    //print_usb(tempBuf, length);
    
	// A match to this will toggle the green LED.
	// Regardless of its value (provided it is less than OCR1A), it will match at the frequency of timer 1.
	OCR1B = 1;

	//Enable output compare match interrupt on timer 1A
    TIMSK1 |= 0x02;


}


//INTERRUPT HANDLERS
ISR(TIMER0_COMPA_vect) {

	// This is the Interrupt Service Routine for Timer0 (10ms clock used for scheduling red).
	// Each time the TCNT count is equal to the OCR0 register, this interrupt is "fired".

	// Increment ticks
	G_ms_ticks++;

	// if time to toggle the RED LED, set flag to release
	if ( ( G_ms_ticks % G_red_period ) == 0 )
		G_release_red = 1;
}


