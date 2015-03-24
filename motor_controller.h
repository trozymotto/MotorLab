#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include <avr/io.h>         //gives us names for registers
#include <avr/interrupt.h>

#include <inttypes.h> //gives us uintX_t

#include <string.h>

/*
typedef struct {
    volatile int currEncA;
    volatile int currEncB;
    volatile int lastEncA;
    volatile int lastEncB;
    volatile int encoder;
    volatile double speed;
    char reverse;
}MotorData;
*/

// define the data direction registers
#define DD_REG_ENC  DDRD
#define DD_REG_PWM  DDRD
#define DD_REG_DIR  DDRC

// define the output ports by which you send signals to the LEDs
#define PORT_M1_2        PORTD
#define PORT_M1_2_DIR    PORTC

// define the bit-masks for each port that the LEDs are attached to
#define BIT_M1_ENCA   ( 1 << 2 )
#define BIT_M1_ENCB   ( 1 << 3 )
#define BIT_M1_PWM    ( 1 << 7 )
#define BIT_M1_DIR    ( 1 << 7 ) 

#define BIT_M2_ENCA   ( 1 << 0 )
#define BIT_M2_ENCB   ( 1 << 1 )
#define BIT_M2_PWM    ( 1 << 6 )
#define BIT_M2_DIR    ( 1 << 6 ) 

// define "function" calls for getting the encoder pins
#define GET_M1_ENCA ((PORT_M1_2 & BIT_M1_ENCA) > 0)?1:0
#define GET_M1_ENCB ((PORT_M1_2 & BIT_M1_ENCB) > 0)?1:0

#define GET_M2_ENCA ((PORT_M1_2 & BIT_M2_ENCA) > 0)?1:0
#define GET_M2_ENCB ((PORT_M1_2 & BIT_M2_ENCB) > 0)?1:0


// function call prototypes
void init_encoder();
void init_motor_control();
void motor_test();
void clear_motor();

#endif

