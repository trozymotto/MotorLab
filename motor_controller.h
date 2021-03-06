#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include <avr/io.h>         //gives us names for registers
#include <avr/interrupt.h>

#include <inttypes.h> //gives us uintX_t

#include <string.h>

typedef enum {
    CTRL_OFF,
    CTRL_SPD_SETPNT,
    CTRL_SPD_INTERP,
    CTRL_POS_SETPNT,
    CTRL_POS_INTERP,
    CTRL_MAX_TYPES
}eControlType;

typedef enum {
    NO_CHANGE,
    INCREMENT,
    DECREMENT
}eChangeType;

typedef enum {
    NO_PARAM,
    KP_SPD,
    KI_SPD,
    KD_SPD,
    KP_POS,
    KI_POS,
    KD_POS
}eParamType;

// Kp,Ki,Kd,Velocity,Setpoint,Position,Torque
// "%f,%f,%f,%f,%d,%d,%d
typedef struct {
    unsigned long Time;
    double Ki;
    double Kp;
    double Kd;
    double Velocity;
    int Setpoint;
    int Position;
    int Torque;
}sLogData;


#define COUNTS_PER_REV 2249
#define FULL_REV 360

// define the data direction registers
#define DD_REG_ENC  DDRD
#define DD_REG_PWM  DDRD
#define DD_REG_DIR  DDRC

// define the output ports by which you send signals to the LEDs
#define PORT_M1_2        PORTD
#define PORT_M1_2_DIR    PORTC

// define the input ports
#define PIN_M1_2      PIND
#define PIN_M1_2_DIR  PINC

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
#define GET_M1_ENCA ((PIN_M1_2 & BIT_M1_ENCA) > 0)?1:0
#define GET_M1_ENCB ((PIN_M1_2 & BIT_M1_ENCB) > 0)?1:0

#define GET_M2_ENCA ((PIN_M1_2 & BIT_M2_ENCA) > 0)?1:0
#define GET_M2_ENCB ((PIN_M1_2 & BIT_M2_ENCB) > 0)?1:0


// function call prototypes
void init_encoder();
void init_motor_control();
void motor_pwm_init();
void motor_test();
void clear_motor();
int limit_value(int data, int lower, int upper);
int abs_int(int value);
void interpolator();
void speed_interpolator();
void motor_speed(int motor, int speed);

// Access routines for Menu control
void set_ctrl_type(eControlType type, int setpoint);
void print_motor_vals();
void enable_logging(int enable);
void change_value(eParamType param, eChangeType change);
eControlType get_mode();
#endif

