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
char send_buffer[32];

int main(void) 
{
	// Initialization here.
    //encoders_init(IO_D2, IO_D3, IO_C0, IO_B4);
    //init_encoder();
    init_motor_control();
	lcd_init_printf();	// required if we want to use printf() for LCD printing
	//init_LEDs();
	init_timers();
	//init_menu();	// this is initialization of serial comm through USB
	
	clear();	// clear the LCD

	//enable interrupts
	sei();
	
	while (1) 
	{
        //lab1_test();
        motor_test();
		
	} //end while loop
} //end main

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

