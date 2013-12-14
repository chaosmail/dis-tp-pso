#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <math.h>

#include "./codec/e_sound.h"
#include "./motor_led/e_init_port.h"
#include "./motor_led/e_led.h"
#include "./motor_led/e_motors.h"
#include "./uart/e_uart_char.h"
//#include "./a_d/advance_ad_scan/e_acc.h"
#include "./a_d/advance_ad_scan/e_prox.h"
#include "./a_d/advance_ad_scan/e_ad_conv.h"

/********** Constants **********/

#define MAX_SPEED 1000.0 // Maximum speed of wheels in each direction
#define MAX_ACC 1000.0 // Maximum amount speed can change in 128 ms



// Waits for a certain amount of time
// Note that the effective waiting time is not always the same (because of possible interrupts).
void wait(unsigned long num) {
	while (num > 0) {num--;}
}

/********** Function declarations **********/

//double fitfunc(double[],int);
double rnd();
double unf(double, double);

// Main program
int main() {

	int speedl = 0;
	int speedr = 0;
	int oldSpeedl = 0;
	int oldSpeedr = 0;
   
	int length = 19;
	int counter = 0;
	    
	    int trajl[19] = {6,9,9,6,
		              	7,6,3,3,
		           	6,6,6,3,
				3,3,6,6,
				6,3,3};
		             
	    int trajr[19] = {6,3,3,6,
		              	7,6,9,9,
		              	6,6,6,9,
				9,9,6,6,
				6,9,9};


	// Initialize system and sensors
	e_init_port();
	e_init_uart1();
	e_init_motors();
	e_init_ad_scan();

	// Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR = 0;
		__asm__ volatile ("reset");
	}

	// Run
	while (1) {
		oldSpeedl = speedl;
        	oldSpeedr = speedr;

		// Leader just moves ranomly
		// weighted movement
		//speedl = (int) ((MAX_SPEED*unf(0.6,1))*7 + oldSpeedl*3)/10;
		//speedr = (int) ((MAX_SPEED*unf(0.6,1))*7 + oldSpeedr*3)/10;
		/*speedl = (int) (MAX_SPEED*rand());
		speedr = (int) (MAX_SPEED*rand());*/

		speedl=100*trajl[counter%length];
		speedr=100*trajr[counter%length];	

		// Set the motor speeds
		e_set_speed_left(speedl);
		e_set_speed_right(speedr);
		
		
		// Indicate with leds on which side we are turning (leds are great for debugging) 
		if (speedl>speedr) {
			e_set_led(1, 1);
			e_set_led(7, 0);
		} else {
			e_set_led(1, 0);
			e_set_led(7, 1);
		}

		// Wait for some time
		//wait(100000); // for random
		wait(5*100000); // for predefined

		counter++;
	}

	return 0;
}

// Generate random number from 0 to 1
double rnd() {
    return (double)rand()/RAND_MAX;
}

double unf(double min, double max) {
    return min + rnd()*(max-min);
}
