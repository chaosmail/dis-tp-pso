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

#define RND_OLD_WEIGHT 2
#define RND_NEW_WEIGHT 8
#define RND_MIN_SPEED 0.2
#define RND_MAX_SPEED 1.0

#define TRAJ_LEN 19
#define TRAJ_STEPS 5
#define AVD_STEPS 2
#define SENSOR_THRESHOLD 300
#define SESOR_BIAS 30

#define STATE_DETERMINISTIC 0 // Leader on a predefined trajectory
#define STATE_RANDOM 1 // Leader on a randomized trajectory
#define STATE_OBSTACLE 2 // Leader in obstacle avoidance mode

#define STATE STATE_RANDOM

// Waits for a certain amount of time
// Note that the effective waiting time is not always the same (because of possible interrupts).
void wait(unsigned long num) {
	while (num > 0) {num--;}
}

/********** Function declarations **********/

//double fitfunc(double[],int);
double rnd();
double unf(double, double);


// Weights for the Braitenberg obstacle avoidance algorithm
int weightleft[8] = {-10, -10, -5, 0, 0, 5, 10, 10};
int weightright[8] = {10, 10, 5, 0, 0, -5, -10, -10};


// Main program
int main() {

	int speedl = 0;
	int speedr = 0;
	int oldSpeedl = 0;
	int oldSpeedr = 0;
   
	int counter = 0;
	int step = 0;
	int avdStep = 0;
	int state = STATE;
    	int oldState = STATE;
	int sensor[8], value, maxSensorValue;
	int i;

	int trajl[TRAJ_LEN] =   {6,9,9,6,
			      	 7,6,3,3,
			   	 6,6,6,3,
				 3,3,6,6,
				 6,3,3};
			     
	int trajr[TRAJ_LEN] =   {6,3,3,6,
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

		speedl = 0;
		speedr = 0;

		// Get sensor values
		for (i = 0; i < 8; i++) {
			sensor[i] = e_get_prox(i);
		}
		
		maxSensorValue = 0;
		if (sensor[0]!=0 && sensor[0] > maxSensorValue) maxSensorValue = sensor[0];
		if (sensor[1]!=0 && sensor[1] > maxSensorValue) maxSensorValue = sensor[1];
		if (sensor[6]!=0 && sensor[6] > maxSensorValue) maxSensorValue = sensor[6];
		if (sensor[7]!=0 && sensor[7] > maxSensorValue) maxSensorValue = sensor[7];

		// change state to OBSTACLE MODE
		if (maxSensorValue > SENSOR_THRESHOLD && state!=STATE_OBSTACLE) {
			oldState = state;
			state = STATE_OBSTACLE;
			avdStep = 0;
		}
		// back to old state
		else if (avdStep >= AVD_STEPS) {
			state = oldState;
			step = 0;
			avdStep = 0;
		}

		
		if (state==STATE_OBSTACLE) {

			// Add the weighted sensors values
			for (i = 0; i < 8; i++) {
				if (i!=4 && i!=5) {
					value = (sensor[i] >> 4);
					speedl += weightleft[i] * value;
					speedr += weightright[i] * value;
				}
			}

			speedl += 2*SESOR_BIAS;
			speedr += 2*SESOR_BIAS;

			avdStep++;
		}
		else if (state==STATE_RANDOM) {

			// Do random trajectory
			if (step >= TRAJ_STEPS) {
				speedl = (int) ((MAX_SPEED*unf(RND_MIN_SPEED,RND_MAX_SPEED))*RND_NEW_WEIGHT + oldSpeedl*RND_OLD_WEIGHT)/(RND_NEW_WEIGHT+RND_OLD_WEIGHT);
        			speedr = (int) ((MAX_SPEED*unf(RND_MIN_SPEED,RND_MAX_SPEED))*RND_NEW_WEIGHT + oldSpeedr*RND_OLD_WEIGHT)/(RND_NEW_WEIGHT+RND_OLD_WEIGHT);
				
				step = 0;
			}
			else {
				speedl = oldSpeedl;
        			speedr = oldSpeedr;

				step++;
			}
		}		
		else {
	
			// Do deterministic trajectory
			if (step >= TRAJ_STEPS) {
				speedl = 100*trajl[counter%TRAJ_LEN];
				speedr = 100*trajr[counter%TRAJ_LEN];

				counter++;
				step = 0;
			}
			else {
				speedl = oldSpeedl;
        			speedr = oldSpeedr;

				step++;
			}
		}
		
		// Set the motor speeds
		e_set_speed_left(speedl);
		e_set_speed_right(speedr);
		
		// Indicate with leds on which side we are turning
		if (speedl>speedr) {
			e_set_led(1, 1);
			e_set_led(7, 0);
		} else {
			e_set_led(1, 0);
			e_set_led(7, 1);
		}

		// Wait for some time
		wait(100000);
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
