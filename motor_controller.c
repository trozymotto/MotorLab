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

// Make a string array so we can print out the mode
const char * TypeStrings[] = {
    "CTRL_OFF",
    "CTRL_SPD_SETPNT",
    "CTRL_SPD_INTERP",
    "CTRL_POS_SETPNT",
    "CTRL_POS_INTERP"
};

// Timing measurement variables
unsigned long time1 = 0;
unsigned long time2 = 0;
unsigned long timeDiff = 0;

// GLOBALS
eControlType m1CtrlType = CTRL_OFF;
eControlType m2CtrlType = CTRL_OFF;

// Log Data
#define LOG_PERIOD 10
#define MAX_LOG_LEN 200
volatile int logIndex = 0;
sLogData log_data[MAX_LOG_LEN];
volatile int bEnableLog = 0;
eControlType loggedType = CTRL_OFF;

// Global variables
volatile int m1SpdSetpoint;
volatile int m1PosSetpoint;

volatile int m1currEncA = 0;
volatile int m1currEncB = 0;
volatile int m1lastEncA = 0;
volatile int m1lastEncB = 0;
volatile signed long m1encoder = 0;

volatile int m1SpdSetpointArray[10] = {300, 600, 500, 300, 50, -100, -300, -500, -200, 100};
volatile int m1SpdSettimeArray[10] = {10, 10, 5, 10, 10, 10, 50, 10, 10, 20};
volatile int m1PosSetpointArray[10] = {300, 1000, 500, 1000, 1500, 1000, 300, -200, -500, 0};
volatile int m1PosSettimeArray[10] = {100, 1000, 1000, 1000, 1000, 1000, 500, 1000, 1000, 200};
volatile int m1torque = 0;
volatile int m1test = 0;
volatile int m1LastSpeedCnt = 0;
volatile int m1PosError = 0;
volatile int m1SpdError = 0;
volatile double m1LastSpd = 0;
volatile int m1LastPos = 0;
volatile int m1LastPosError = 0;
volatile int m1LastSpdError = 0;
volatile int m1PosIndex = 0;
volatile int m1SpdIndex = 0;

// PID values
volatile double m1Kp = 0.2;
volatile double m1Ki = 0.0015;
volatile double m1Kd = 5;
volatile double m1SpdKp = 0.2;
volatile double m1SpdKi = 0.015;
volatile double m1SpdKd = 0.0;
volatile int m1d = 0;
volatile int m1i = 0;
volatile int m1Spd_d = 0;
volatile int m1Spd_i = 0;
volatile int m1settle = 0;
double m1Velocity = 0.0;
double m1Accel = 0.0;

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
    m1torque = 0;
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
	    if(m1torque > 0)
		    m1torque += 10;
		else
		    m1torque -= 10;
	}
	
    // Reverse direction
    if(button_is_pressed(MIDDLE_BUTTON))
	{
	    // Stop first then change direction
	    set_motors(0,0);
	    delay_ms(200);
	    m1torque *= -1;
	}

	if(button_is_pressed(BOTTOM_BUTTON))
	{
		if(m1torque > 0)
		    m1torque -= 10;
		else
		    m1torque += 10;
	}

	m1torque = limit_value(m1torque, -255, 255);

	motor_speed(1, m1torque);
	delay_ms(50);

    // Read the counts for motor 1 and print to LCD.
    lcd_goto_xy(0,0);
    print_long(0);//m1encoder);
    print(", ");
    print_long(m1torque);
    print(", ");
    print_long(timeDiff);
    print(", ");
    lcd_goto_xy(0,1);
    //print_long((signed long)m1Velocity*1000);
    print_long(m1PosError);
    print(", ");
    print_long(m1Velocity);//m1i);
    print(", ");
    print_long(m1SpdError);//m1d);
    print(" ");
    
}


// External Interrupt 1 Handler
ISR(PCINT3_vect)
{
    m1currEncA = GET_M1_ENCA;
    m1currEncB = GET_M1_ENCB;
    
    char m1plus = m1currEncA ^ m1lastEncB;
    char m1minus = m1currEncB ^ m1lastEncA;
    
    if(m1plus)
        m1encoder++;
    if(m1minus)
        m1encoder--;

    // Look at the time for one revolution
    if(m1encoder == 0 || m1encoder%COUNTS_PER_REV == 0)
    {
        time2 = get_ms();
        timeDiff = time2 - time1;
        time1 = time2;
    }
    
    m1lastEncA = m1currEncA;
    m1lastEncB = m1currEncB;
}

// POSITION CONTROL and SPEED CONTROL ISR
#define SPEED_PERIOD 100
ISR(TIMER0_COMPA_vect) 
{
    static int logCnt = 0;
    static int speedCnt = 0;
    
    int m1enc = m1encoder;
    
    speedCnt++;
    // Calculate the velocity
    if(speedCnt >= SPEED_PERIOD)
    {
        m1Velocity = (double)(m1enc - m1LastSpeedCnt);///0.001;
        m1LastSpeedCnt = m1enc;
        
        if(!(m1CtrlType == CTRL_SPD_SETPNT || m1CtrlType == CTRL_SPD_INTERP)){
            speedCnt = 0;
        }
    }
    
    // SPEED CONTROL
    switch(m1CtrlType)
    {
    case CTRL_SPD_SETPNT:
    case CTRL_SPD_INTERP:
        if(speedCnt >= SPEED_PERIOD)
        {   
            m1SpdError = m1SpdSetpoint - m1Velocity;
            m1Accel = m1LastSpd - m1Velocity;
            m1Spd_i += m1PosError;
            
            // Speed Control PID Equation
            m1torque = m1torque + (int)(m1SpdError * m1SpdKp) + (int)(m1Spd_i * m1SpdKi) - (int)(m1Accel * m1SpdKd);
            m1LastSpd = m1Velocity;
            
            // Handle reseting the I component
            if(m1SpdError == 0)
                m1Spd_i = 0;
            
            // Run the interpolator
            if(m1CtrlType == CTRL_SPD_INTERP)
                speed_interpolator();
            
            speedCnt = 0;
        }
        break;
    // POSITION CONTROL
    case CTRL_POS_SETPNT:
    case CTRL_POS_INTERP:
        m1PosError = m1PosSetpoint - m1encoder; 
        m1d = m1LastPos - m1encoder;
        m1i += m1PosError; 
        
        // Position Control PID Equation
        m1torque = (int)(m1PosError * m1Kp) + (int)(m1i * m1Ki) - (int)(m1d * m1Kd); 
        
        m1LastPos = m1encoder;
        
        // Handle reseting the I component
        if(m1PosError == 0)
            m1i = 0;
                
        // Run the interpolator
        if(m1CtrlType == CTRL_POS_INTERP)
            interpolator();
        break;
    default:
        break;
    }
    
    // Logging data
    if(bEnableLog)
    {
        if(!(m1CtrlType == CTRL_OFF)){
            loggedType = m1CtrlType;
        }
        logCnt++;
        if(logCnt >= LOG_PERIOD)
        {
            logCnt = 0;
            // Put the data in the buffer if there is room
            if(logIndex < MAX_LOG_LEN)
            {
                switch(m1CtrlType)
                {
                case CTRL_SPD_SETPNT:
                case CTRL_SPD_INTERP:
                    log_data[logIndex].Time = get_ms();
                    log_data[logIndex].Kp = m1SpdKp;
                    log_data[logIndex].Ki = m1SpdKi;
                    log_data[logIndex].Kd = m1SpdKd; 
                    log_data[logIndex].Velocity = m1Velocity;
                    log_data[logIndex].Setpoint = m1SpdSetpoint;
                    log_data[logIndex].Position = m1encoder;
                    log_data[logIndex].Torque = m1torque;
                    logIndex++;
                    break;
                case CTRL_POS_SETPNT:
                case CTRL_POS_INTERP:
                    log_data[logIndex].Time = get_ms();
                    log_data[logIndex].Kp = m1Kp;
                    log_data[logIndex].Ki = m1Ki;
                    log_data[logIndex].Kd = m1Kd; 
                    log_data[logIndex].Velocity = m1Velocity;
                    log_data[logIndex].Setpoint = m1PosSetpoint;
                    log_data[logIndex].Position = m1encoder;
                    log_data[logIndex].Torque = m1torque;
                    logIndex++;
                    break;
                default:
                    break;
                }
            }
            // Disable log and print if buffer full
            else
                enable_logging(0);
        }
    }
}

void set_ctrl_type(eControlType type, int setpoint)
{
    m1CtrlType = type;
    switch (type)
    {
    case CTRL_SPD_SETPNT:
        m1SpdSetpoint = setpoint;
        break;
    case CTRL_POS_SETPNT:
        m1PosSetpoint = setpoint;
        break;
    case CTRL_SPD_INTERP:
    case CTRL_POS_INTERP:
        m1settle = 0;
        m1SpdIndex = 0;
        m1PosIndex = 0;
        break;
    default:
        break;
    }
}

void speed_interpolator()
{
    if(m1CtrlType == CTRL_SPD_SETPNT)
    {
        if(abs_int(m1SpdError) <= 10)
        {
            m1settle++;
        }
        if(m1settle > m1SpdSettimeArray[m1SpdIndex])
        {
            m1settle = 0;
            m1SpdIndex++;
            if(m1SpdIndex >= 10)
                m1SpdIndex = 0;
        }
        m1SpdSetpoint = m1SpdSetpointArray[m1SpdIndex];
    }
}

void interpolator()
{
    if(m1CtrlType == CTRL_POS_SETPNT)
    {
        if(abs_int(m1PosError) <= 15)
        {
            m1settle++;
        }
        if(m1settle > m1PosSettimeArray[m1PosIndex])
        {
            m1settle = 0;
            m1PosIndex++;
            if(m1PosIndex >= 10)
                m1PosIndex = 0;
        }
        m1PosSetpoint = m1PosSetpointArray[m1SpdIndex];
    }
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
    TCCR2A = (1<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (1<< WGM21) | (1<<WGM20); //0x83;

    // use the system clock/8 (=2.5 MHz) as the timer clock,
    // which will produce a PWM frequency of 10 kHz
    //Page 153
    TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (0<<WGM22) |  (0<<CS22) | (1<<CS21) | (0 << CS20); //0x02;

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

#define KP_CHANGE_AMT 0.001
#define KI_CHANGE_AMT 0.001
#define KD_CHANGE_AMT 0.1
void change_value(eParamType param, eChangeType change)
{
    switch(param)
    {
    case KP_SPD:
        if(change == INCREMENT)
            m1SpdKp += KP_CHANGE_AMT;
        else
            m1SpdKp -= KP_CHANGE_AMT;
        if(m1SpdKp < 0)
            m1SpdKp = 0;
        break;
    case KI_SPD:
        if(change == INCREMENT)
            m1SpdKi += KI_CHANGE_AMT;
        else
            m1SpdKi -= KI_CHANGE_AMT;
        if(m1SpdKi < 0)
            m1SpdKi = 0;
        break;
    case KD_SPD:
        if(change == INCREMENT)
            m1SpdKd += KD_CHANGE_AMT;
        else
            m1SpdKd -= KD_CHANGE_AMT;
        if(m1SpdKd < 0)
            m1SpdKd = 0;
        break;
    case KP_POS:
        if(change == INCREMENT)
            m1Kp += KP_CHANGE_AMT;
        else
            m1Kp -= KP_CHANGE_AMT;
        if(m1Kp < 0)
            m1Kp = 0;
        break;
    case KI_POS:
        if(change == INCREMENT)
            m1Ki += KI_CHANGE_AMT;
        else
            m1Ki -= KI_CHANGE_AMT;
        if(m1Ki < 0)
            m1Ki = 0;
        break;
    case KD_POS:
        if(change == INCREMENT)
            m1Kd += KD_CHANGE_AMT;
        else
            m1Kd -= KD_CHANGE_AMT;
        if(m1Kd < 0)
            m1Kd = 0;
        break;
    default:
        break;
    }
}

void print_motor_vals()
{
    // Used to pass to USB_COMM for serial communication
	int length;
	char tempBuffer[64];
    wait_for_sending_to_finish();
    
    if( get_mode() == CTRL_SPD_SETPNT || get_mode() == CTRL_SPD_INTERP || get_mode() == CTRL_OFF)
    {
        // Print the header
        length = sprintf( tempBuffer, "SPD: Kp,Ki,Kd,Vm,Pr,Pm,T\r\n");
        print_usb(tempBuffer, length);
        wait_for_sending_to_finish();
        
        // View the current values Kd, Kp, Vm, Pr, Pm, and T
        length = snprintf( tempBuffer, 64, "%f,%f,%f,%f\r\n",//,%d,%d,%d\r\n", 
                      m1SpdKp, m1SpdKi, m1SpdKd, m1Velocity );
        print_usb(tempBuffer, length);
        wait_for_sending_to_finish();
    }
	if( get_mode() == CTRL_POS_SETPNT || get_mode() == CTRL_POS_INTERP || get_mode() == CTRL_OFF)
	{
        // Print the header
        length = sprintf( tempBuffer, "POS: Kp,Ki,Kd,Vm,Pr,Pm,T\r\n");
        print_usb(tempBuffer, length);
        wait_for_sending_to_finish();
        
        // View the current values Kd, Kp, Vm, Pr, Pm, and T
        length = snprintf( tempBuffer, 64, "%f,%f,%f,%f\r\n",//,%d,%d,%d\r\n", 
                      m1Kp, m1Ki, m1Kd, m1Velocity );
        print_usb(tempBuffer, length);
        wait_for_sending_to_finish();
    }
}

void enable_logging(int enable)
{
   int i;
   
   // Used to pass to USB_COMM for serial communication
   int length;
   char tempBuffer[64];
   wait_for_sending_to_finish();
   
   // clear out any previous data
   if(enable)
   {
       logIndex = 0;
       for( i = 0; i < MAX_LOG_LEN; i++)
           memset(&log_data[i],0,sizeof(sLogData));
   }
   else
   {
       if(logIndex > MAX_LOG_LEN)
           logIndex = 0;
       // Print the data to the terminal
       // Kp,Ki,Kd,Velocity,Setpoint,Position,Torque
       // "%f,%f,%f,%f,%d,%d,%d
       // Print the header
       length = snprintf( tempBuffer, 64, "Logged: %s\r\n", TypeStrings[loggedType]);
       print_usb(tempBuffer, length);
       wait_for_sending_to_finish();
       // Print the header
       length = snprintf( tempBuffer, 64, "Time,Kp,Ki,Kd,Velocity,Setpoint,Position,Torque\r\n");
       print_usb(tempBuffer, length);
       wait_for_sending_to_finish();
       // Print the data
       for( i = 0; i < logIndex; i++)
       {
           length = snprintf( tempBuffer, 64, "%ul,%f,%f,%f,%f,%d,%d,%d\r\n", 
                      log_data[i].Time, log_data[i].Kp, log_data[i].Ki, log_data[i].Kd, 
                      log_data[i].Velocity, log_data[i].Setpoint, 
                      log_data[i].Position, log_data[i].Torque );
           print_usb(tempBuffer, length);
           wait_for_sending_to_finish();
       }
   }
   bEnableLog = enable;
}

eControlType get_mode()
{
    return m1CtrlType;
}

