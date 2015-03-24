//#include <pololu/orangutan.h>

#include "LEDs.h"
#include "timer.h"
#include "menu.h"
#include "motor_controller.h"

//Gives us uintX_t (e.g. uint32_t - unsigned 32 bit int)
//On the ATMega128 int is actually 16 bits, so it is better to use
//  the int32_t or int16_t so you know exactly what is going on
#include <inttypes.h> //gives us uintX_t

// useful stuff from libc
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// GLOBALS
volatile int m1currEncA = 0;
volatile int m1currEncB = 0;
volatile int m1lastEncA = 0;
volatile int m1lastEncB = 0;
volatile int m1encoder = 0;
volatile int m1speed = 0;
volatile int m1reverse = 0;

volatile int m2currEncA = 0;
volatile int m2currEncB = 0;
volatile int m2lastEncA = 0;
volatile int m2lastEncB = 0;
volatile int m2encoder = 0;
volatile int m2speed = 0;
volatile int m2reverse = 0;

volatile int sendLen = 0;
char send_buffer[32];
// END Motor specific items

void clear_motors()
{
    m1currEncA = 0;
    m1currEncB = 0;
    m1lastEncA = 0;
    m1lastEncB = 0;
    m1encoder = 0;
    m1speed = 0;
    m1reverse = 0;

    m2currEncA = 0;
    m2currEncB = 0;
    m2lastEncA = 0;
    m2lastEncB = 0;
    m2encoder = 0;
    m2speed = 0;
    m2reverse = 0;
}

void init_motor_control()
{
    clear_motors();
    init_encoder();
}

void init_encoder()
{
    // Set as inputs
    // Encoder lines are PORTD0-3
	DD_REG_ENC &= ~(BIT_M1_ENCA | BIT_M1_ENCB | BIT_M2_ENCA | BIT_M2_ENCB);
	
	// Grab the intial encoder bit values
    m1lastEncA = GET_M1_ENCA;
    m1lastEncB = GET_M1_ENCB;
    m2lastEncA = GET_M2_ENCA;
    m2lastEncB = GET_M2_ENCB;
	
	// Setup the pin change interrupts for the pins
	//EIMSK &= ~();
	//EICRA = 0x14;  // ISC mode is set to interrupt on both edges
	PCMSK3 = 0x0F; // Enable PCINT24-27 in bank 3
	PCICR = 0x08;  // Enable pin change interrupt 3
}


void motor_test()
{
	if(button_is_pressed(TOP_BUTTON))
	{	
		m1speed += 10;
		m2speed += 10;
	}
	
    // Reverse direction
    if(button_is_pressed(MIDDLE_BUTTON))
	{
	    // Stop first then change direction
	    set_motors(0,0);
	    delay_ms(200);
	    m1reverse = !m1reverse;
	    m2reverse = !m2reverse;
	}

	if(button_is_pressed(BOTTOM_BUTTON))
	{
		m1speed -= 10;
		m2speed -= 10;
	}

	if(m1speed < 0)
		m1speed = 0;

	if(m2speed < 0)
		m2speed = 0;

	if(m1speed > 255)
		m1speed = 255;

	if(m2speed > 255)
		m2speed = 255;

	set_motors(m1speed * (m1reverse ? -1 : 1), m2speed * (m2reverse ? -1 : 1));
	delay_ms(50);

    // Read the counts for motor 1 and print to LCD.
/*    lcd_goto_xy(0,0);
    print_long(m1encoder);
    print(" ");
    lcd_goto_xy(0,1);
    print_long(m2encoder);
    print(" ");*/

}


// External Interrupt 1 Handler
ISR(PCINT3_vect)
{
    m1currEncA = GET_M1_ENCA;
    m1currEncB = GET_M1_ENCB;
    m2currEncA = GET_M2_ENCA;
    m2currEncB = GET_M2_ENCB;
    
    lcd_goto_xy(0,0);
    print_long(m1currEncA);
    print(" ");
    lcd_goto_xy(0,1);
    print_long(m1lastEncA);
    print(" ");
    
    if(m1currEncA != m1lastEncA)
    {
        m1lastEncA = m1currEncA;
        if(m1currEncA == 1)
        {
            if(m1lastEncB == 0)
                m1encoder++;
            else
                m1encoder--;
        }
        else 
        {
            if(m1lastEncB == 1)
                m1encoder++;
            else
                m1encoder--;
        }
    }
    /*
    else if(m1currEncB != m1lastEncB)
    {
        m1lastEncB = m1currEncB;
        if(m1currEncB == 1)
        {
            if(m1lastEncA == 1)
                m1encoder++;
            else
                m1encoder--;
        }
        else 
        {
            if(m1lastEncA == 0)
                m1encoder++;
            else
                m1encoder--;
        }
    }*/
    
    
    if(m2currEncA != m2lastEncA)
    {
        m2lastEncA = m2currEncA;
        if(m2currEncA == 1)
        {
            if(m2lastEncB == 0)
                m2encoder++;
            else
                m2encoder--;
        }
        else 
        {
            if(m2lastEncB == 1)
                m2encoder++;
            else
                m2encoder--;
        }
    }
    /*else if(m2currEncB != m2lastEncB)
    {
        m2lastEncB = m2currEncB;
        if(m2currEncB == 1)
        {
            if(m2lastEncA == 1)
                m2encoder++;
            else
                m2encoder--;
        }
        else 
        {
            if(m2lastEncA == 0)
                m2encoder++;
            else
                m2encoder--;
        }
    }*/
}

/*
// External Interrupt 2 Handler
ISR(PCINT2_vect)
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
*/