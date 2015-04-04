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
volatile int m1SpdSetpoint[10] = {300, 600, 500, 100, -100, -500, 300, -200, -500, 0};
volatile int m1SpdSettime[10] = {10, 10, 5, 10, 10, 10, 50, 10, 10, 20};
volatile int m1setpoint[10] = {300, 1000, 500, 1000, 1500, 1000, 300, -200, -500, 0};
volatile int m1settime[10] = {100, 1000, 1000, 1000, 1000, 1000, 500, 1000, 1000, 200};
volatile int m1speed = 0;
volatile int m1test = 0;
volatile int m1LastSpeedCnt = 0;
volatile int m1PosError = 0;
volatile int m1SpdError = 0;
volatile double m1LastSpd = 0;
volatile int m1LastPos = 0;
volatile int m1LastPosError = 0;
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
    motor_pwm_init();
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
	//set_motors(m1speed, m2speed);
	motor_speed(1, m1speed);
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
    print_long(m1CalcSpeed);//m1i);
    print(", ");
    print_long(m1SpdError);//m1d);
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

//POSITION CONTROL
/*ISR(TIMER0_COMPA_vect) 
{
    m1PosError = m1setpoint[m1index] - m1encoder; 
    m1d = m1LastPos - m1encoder;
    m1i = m1i + m1PosError; 
    m1speed = (int)(m1PosError * m1Kp) + (int)(m1i * m1Ki) - (int)(m1d * m1Kd); 
    
    m1LastPos = m1encoder;
    
    interpolator();
}*/

// SPEED CONTROL
int speedCnt = 0;
ISR(TIMER0_COMPA_vect) 
{
    speedCnt++;

    if(speedCnt >= 100)
    {
        int m1enc = m1encoder;
        m1CalcSpeed = (double)(m1enc - m1LastSpeedCnt);///0.001;
        m1LastSpeedCnt = m1enc;
        
        m1SpdError = m1SpdSetpoint[m1index] - m1CalcSpeed;
    
        m1speed = m1speed + (int)(m1SpdError*0.2);
        m1LastSpd = m1CalcSpeed;
        speed_interpolator();
        speedCnt = 0;
    }
}



void speed_interpolator()
{
    if(abs_int(m1SpdError) <= 10)
    {
        m1settle++;
    }
    if(m1settle > m1SpdSettime[m1index])
    {
        m1settle = 0;
        m1index++;
        if(m1index >= 10)
            m1index = 0;
    }
    m1test = m1settle;
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
    DD_REG_PWM |= BIT_M1_PWM | BIT_M2_PWM;
    DD_REG_DIR |= BIT_M1_DIR | BIT_M2_DIR;
    
    //Fast PWM - TOP =OCRA, Update OCRx at BOTTOM, TOV FLag Set on TOP
    //Mode 7, WGM2 = 1, WGM1 = 1, WGM0 = 1
    //CLEAR OC21 on Compare Match, set OC2A at BOTTOM (non-inverting mode). Table 17-3 Page 151
    //Page 151
    TCCR2A = (1<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<3) | (0<<2) | (1<< WGM21) | (1<<WGM20); //0x83;

    // use the system clock/8 (=2.5 MHz) as the timer clock,
    // which will produce a PWM frequency of 10 kHz
    //Page 153
    TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (0x0<<5) | (0x0<<4) | (0<<WGM22) |  (0<<CS22) | (1<<CS21) | (0 << CS20); //0x02;

    OCR2A = 0;
}

void motor_speed(int motor, int speed)
{
   speed = limit_value(speed, -255, 255);
   if(motor == 1)
   {
      if(speed >= 0)
      {
         PORT_M1_2_DIR |= BIT_M1_DIR;
         OCR2A = speed;
      }
      else
      {
         PORT_M1_2_DIR &= ~BIT_M1_DIR;
         OCR2A = abs_int(speed);
      }
   }
   else
   {
   
   }
}

