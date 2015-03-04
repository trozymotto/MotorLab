#include <pololu/orangutan.h>

#include "LEDs.h"
#include "timer.h"
#include "menu.h"

//Gives us uintX_t (e.g. uint32_t - unsigned 32 bit int)
//On the ATMega128 int is actually 16 bits, so it is better to use
//  the int32_t or int16_t so you know exactly what is going on
#include <inttypes.h> //gives us uintX_t

// useful stuff from libc
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// GLOBALS
volatile uint32_t G_yellow_ticks = 0;
volatile uint32_t G_ms_ticks = 0;

volatile uint16_t G_red_period = 500;
volatile uint16_t G_green_period = 500;
volatile uint16_t G_yellow_period = 500;

volatile uint16_t G_release_red = 0;

volatile uint32_t G_red_toggles = 0;
volatile uint32_t G_green_toggles = 0;
volatile uint32_t G_yellow_toggles = 0;

volatile uint8_t G_flag = 0; // generic flag for debugging

// Motor test specific items
// define the bit-masks for each port that the LEDs are attached to
#define DD_REG_ENCA	 DDRB
#define DD_REG_ENCB  DDRC
#define PORT_ENCA  PORTB
#define PORT_ENCB  PORTC
#define BIT_ENCA   ( 1 << 4 )
#define BIT_ENCB   ( 1 << 0 )

#define GET_ENCA ((PORT_ENCA & BIT_ENCA) == BIT_ENCA)?1:0
#define GET_ENCB ((PORT_ENCB & BIT_ENCB) == BIT_ENCB)?1:0

void init_encoder();
void motor_test();

volatile int currEncA = 0;
volatile int currEncB = 0;
volatile int lastEncA = 0;
volatile int lastEncB = 0;
volatile int encoderVal = 0;
volatile int sendLen = 0;
char send_buffer[32];
// END Motor specific items

int main(void) 
{

	// Initialization here.
    //init_encoder();
	lcd_init_printf();	// required if we want to use printf() for LCD printing
	init_LEDs();
	init_timers();
	init_menu();	// this is initialization of serial comm through USB
	
	clear();	// clear the LCD

	//enable interrupts
	sei();
	
	while (1) 
	{
        lab1_test();
        motor_test();
		
	} //end while loop
} //end main

void init_encoder()
{
    // Set as inputs
    // Encoder lines are PORTC0 and PORTB4
	DD_REG_ENCA &= ~BIT_ENCA;
	DD_REG_ENCB &= ~BIT_ENCB;
	
	// Grab the intial encoder bit values
	lastEncA = GET_ENCA;
	lastEncB = GET_ENCB;
	
	// Setup the pin change interrupts for the pins
	// PC0 is PCINT16 and PB4 is PCINT12
	EICRA = 0x14;  // ISC mode is set to interrupt on both edges
	PCMSK1 = 0x10; // Enable PCINT12 in bank 1
	PCMSK2 = 0x01; // Enable PCINT16 in bank 2 
	PCICR = 0x06;  // Enable pin change interrupts 1 and 2
}

int m1_speed = 0;

void motor_test()
{
	static char m1_back = 0;

	if(button_is_pressed(TOP_BUTTON))
	{	
		m1_speed += 10;
	}
	
    // Reverse direction
    if(button_is_pressed(MIDDLE_BUTTON))
	{
	    // Stop first then change direction
	    set_motors(0,0);
	    delay_ms(200);
	    m1_back = !m1_back;
	}

	if(button_is_pressed(BOTTOM_BUTTON))
	{
		m1_speed -= 10;
	}

	if(m1_speed < 0)
		m1_speed = 0;

	if(m1_speed > 255)
		m1_speed = 255;

	set_motors(m1_speed * (m1_back ? -1 : 1),0);
	delay_ms(50);
	
	// Print the encoder counts
	sendLen = sprintf(send_buffer, "Encoder: %d\r\n", encoderVal);
	print_usb( send_buffer, sendLen );
}

// Test code from Lab1
void lab1_test()
{
    if (G_release_red) 
	{
		LED_TOGGLE(RED);
		G_red_toggles++;
		G_release_red = 0; 
	}

	serial_check();
	check_for_new_bytes_received();
}

// External Interrupt 1 Handler
ISR(INT1_vect)
{
    currEncA = GET_ENCA;
    if(currEncA == 1)
    {
        if(lastEncB == 0)
            encoderVal++;
        else
            encoderVal--;
    }
    else 
    {
        if(lastEncB == 1)
            encoderVal++;
        else
            encoderVal--;
    }
    lastEncA = currEncA;
}

// External Interrupt 2 Handler
ISR(INT2_vect)
{
    currEncB = GET_ENCB;
    if(currEncB == 1)
    {
        if(lastEncA == 1)
            encoderVal++;
        else
            encoderVal--;
    }
    else 
    {
        if(lastEncA == 0)
            encoderVal++;
        else
            encoderVal--;
    }
    lastEncB = currEncB;
}

