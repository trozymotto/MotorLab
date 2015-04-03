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
volatile int m1setpoint[10] = {300, 1000, 500, 1000, 1500, 1000, 300, -200, -500, 0};
volatile int m1settime[10] = {100, 1000, 1000, 1000, 1000, 1000, 500, 1000, 1000, 200};
volatile int m1speed = 0;
volatile int m1test = 0;
volatile int m1LastSpeedCnt = 0;
volatile int m1PosError = 0;
volatile int m1LastPos = 0;
volatile int m1LastPosError = 0;
volatile int m1SpdError = 0;
volatile int m1index = 0;
const double m1Kp = 0.2;
const double m1Ki = 0.0015;
const double m1Kd = 5;
volatile int m1d = 0;
volatile int m1i = 0;
volatile int m1settle = 0;
double m1CalcSpeed = 0.0;

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

    m2currEncA = 0;
    m2currEncB = 0;
    m2lastEncA = 0;
    m2lastEncB = 0;
    m2encoder = 0;
    m2speed = 0;
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
	    if(m1speed > 0)
		    m1speed += 10;
		else
		    m1speed -= 10;
		if(m2speed > 0)
		    m2speed += 10;
		else
		    m2speed -= 10;
	}
	
    // Reverse direction
    if(button_is_pressed(MIDDLE_BUTTON))
	{
	    // Stop first then change direction
	    set_motors(0,0);
	    delay_ms(200);
	    m1speed *= -1;
	    m2speed *= -1;
	}

	if(button_is_pressed(BOTTOM_BUTTON))
	{
		if(m1speed > 0)
		    m1speed -= 10;
		else
		    m1speed += 10;
		if(m2speed > 0)
		    m2speed -= 10;
		else
		    m2speed += 10;
	}

	m1speed = limit_value(m1speed, -255, 255);
	m2speed = limit_value(m2speed, -255, 255);

	//set_motors(m1speed * (m1reverse ? -1 : 1), m2speed * (m2reverse ? -1 : 1));
	set_motors(m1speed, m2speed);
	delay_ms(50);

    // Read the counts for motor 1 and print to LCD.
    lcd_goto_xy(0,0);
    //sendLen = snprintf(send_buffer, 32, "%f", m1CalcSpeed);//(float)m1encoder*FULL_REV/COUNTS_PER_REV);
    //print_from_program_space("you suck!");//send_buffer);
    print_long(m1encoder);
    print(", ");
    print_long(m1speed);
    print(" ");
    lcd_goto_xy(0,1);
    //print_long((signed long)m1CalcSpeed*1000);
    print_long(m1PosError);
    print(", ");
    print_long(m1i);
    print(" ");
    print_long(m1d);
    print(" ");

}


// External Interrupt 1 Handler
ISR(PCINT3_vect)
{
    m1currEncA = GET_M1_ENCA;
    m1currEncB = GET_M1_ENCB;
    m2currEncA = GET_M2_ENCA;
    m2currEncB = GET_M2_ENCB;
    
    char m1plus = m1currEncA ^ m1lastEncB;
    char m1minus = m1currEncB ^ m1lastEncA;
    char m2plus = m2currEncA ^ m2lastEncB;
    char m2minus = m2currEncB ^ m2lastEncA; 
    
    if(m1plus)
        m1encoder++;
    if(m1minus)
        m1encoder--;
    if(m2plus)
        m2encoder++;
    if(m2minus)
        m2encoder--;

    m1lastEncA = m1currEncA;
    m1lastEncB = m1currEncB;
    m2lastEncA = m2currEncA;
    m2lastEncB = m2currEncB;
}

//INTERRUPT HANDLERS
ISR(TIMER0_COMPA_vect) 
{
    int m1enc = m1encoder;
    m1CalcSpeed = (m1encoder - m1LastSpeedCnt) * 100 * FULL_REV/COUNTS_PER_REV;
    m1LastSpeedCnt = m1enc;
    
    m1PosError = m1encoder - m1setpoint[m1index]; 
    m1d = m1encoder - m1LastPos;
    m1i = /*limit_value(*/m1i + m1PosError/*, -2000, 2000)*/; 
    m1speed = (int)(m1PosError * m1Kp) + (int)(m1i * m1Ki) - (int)(m1d * m1Kd); 
    
    m1LastPos = m1encoder;
    
    interpolator();
}

void interpolator()
{
    if(abs_int(m1PosError) <= 15)
    {
        m1settle++;
    }
    if(m1settle > m1settime[m1index])
    {
        m1settle = 0;
        m1index++;
        if(m1index >= 10)
            m1index = 0;
    }
    m1test = m1settle;
}

int limit_value(int data, int lower, int upper)
{
    if(data < (lower))
    {
        return (lower);
    }
    else if(data > upper)
    {
        return upper;
    }
    return data;
}

int abs_int(int value)
{
    if(value < 0)
    {
        return 0-value;
    }
    return value;
}

// Initialize the timers for the PWM control of the two motors
void motor_pwm_init()
{
    TCCR2A = 0x82;
    TCCR2A = 0x04;
}

