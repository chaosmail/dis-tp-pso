#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"
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
#define NB_ALL_SENSOR 8 // Number of all proximity sensors
#define NB_SENSOR 6 // Number of used proximity sensors
#define DATASIZE NB_SENSOR+3 // Number of elements in particle

// Weights for the Braitenberg obstacle avoidance algorithm
double weights[DATASIZE] = { -24.45, 4.96, 1.69, -0.43, 5.70, 100.01, 56.63, -71.26, 17.65 };

// Waits for a certain amount of time
// Note that the effective waiting time is not always the same (because of possible interrupts).
void wait(unsigned long num) {
	while (num > 0) {num--;}
}

// Map the nonlinear sensor values to linear distances
double getRealDistance(double sensorValue) {

    double a = 3766;
    double b = -2.012;
    double offset = 35;

    //return a*exp(b*sensorValue) + offset; //inverse function
    if (sensorValue<=offset) //further than sensors can measure
      return 0;
  
    return log((sensorValue-offset)/a)/b;
}

// Map the Value*Weights to Speed
double getRealSpeed(double val){

    double convVal = 0;
    double mapVal = 2200;

    if (val > mapVal)
        convVal = 1;
    else if (val < -mapVal)
        convVal = -1;
    else if (val != 0) 
        convVal = val/mapVal;

    return MAX_SPEED*convVal;
}

// Main program
int main() {
	char buffer[80];
	//int leftwheel, rightwheel;
	//int sensor[8], value;
	
	
	double left_speed,right_speed; // Wheel speeds
	double old_left, old_right; // Previous wheel speeds (for recursion)
	//int left_encoder,right_encoder;
	double ds_value[NB_ALL_SENSOR];
	int i; // Loop vars

	old_left = 0.0;
   	old_right = 0.0;

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

	// Say hello
	// You'll receive these messages in Minicom. They are therefore well suited for debugging.
	// Note, however, that sending such messages is time-consuming and can slow down your program significantly.
	sprintf(buffer, "Braitenberg starting\r\n");
	e_send_uart1_char(buffer, strlen(buffer));

	// Run the braitenberg algorithm
	while (1) {
		/*// Forward speed
		leftwheel = 200;
		rightwheel = 200;*/

		// Get sensor values
		for (i = 0; i < 8; i++) {
			ds_value[i] = getRealDistance(e_get_prox(i));
			//sprintf(buffer, "%d, ", sensor[i]);
			//e_send_uart1_char(buffer, strlen(buffer));
		}

		// Feed proximity sensor values to neural net
        	left_speed  = weights[0]*ds_value[0] + weights[1]*ds_value[1] + weights[2]*ds_value[2] + weights[3]*ds_value[5] + weights[4]*ds_value[6] + weights[5]*ds_value[7];
        	right_speed = weights[0]*ds_value[7] + weights[1]*ds_value[6] + weights[2]*ds_value[5] + weights[3]*ds_value[2] + weights[4]*ds_value[1] + weights[5]*ds_value[0];		
				
		// Adjust Values to 'normal' ones, that we can use recursive and BIAS
		left_speed *= 10.0;
		right_speed *= 10.0;

		// Recursive connections
		left_speed += weights[NB_SENSOR+1]*(old_left+MAX_SPEED)/(2*MAX_SPEED);
		left_speed += weights[NB_SENSOR+2]*(old_right+MAX_SPEED)/(2*MAX_SPEED);
		right_speed += weights[NB_SENSOR+2]*(old_left+MAX_SPEED)/(2*MAX_SPEED);
		right_speed += weights[NB_SENSOR+1]*(old_right+MAX_SPEED)/(2*MAX_SPEED);

		// Add neural thresholds
		// BIAS
		left_speed += weights[NB_SENSOR];
		right_speed += weights[NB_SENSOR];

		//Map from -1 to 1
		left_speed = getRealSpeed(left_speed);
		right_speed = getRealSpeed(right_speed);

		 // Make sure we don't accelerate too fast
		if (left_speed - old_left > MAX_ACC) left_speed = old_left+MAX_ACC;
		if (left_speed - old_left < -MAX_ACC) left_speed = old_left-MAX_ACC;
		if (right_speed - old_right > MAX_ACC) left_speed = old_right+MAX_ACC;
		if (right_speed - old_right < -MAX_ACC) left_speed = old_right-MAX_ACC;

		// Make sure speeds are within bounds
		if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
		if (left_speed < -1.0*MAX_SPEED) left_speed = -1.0*MAX_SPEED;
		if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
		if (right_speed < -1.0*MAX_SPEED) right_speed = -1.0*MAX_SPEED;// Set new old speeds
		
		old_left = left_speed;
		old_right = right_speed;

		// What is encoders in follower.c???		

		// Set the motor speeds
		//wb_differential_wheels_set_speed((int)left_speed,(int)right_speed);
		e_set_speed_left(left_speed);
		e_set_speed_right(right_speed);		
		//robot_step(128); // run one step		

		/*// Add the weighted sensors values
		for (i = 0; i < 8; i++) {
			value = (sensor[i] >> 4);
			leftwheel += weightleft[i] * value;
			rightwheel += weightright[i] * value;
		}*/

		//sprintf(buffer, " -> desired speed: %d %d\r\n", leftwheel, rightwheel);
		//e_send_uart1_char(buffer, strlen(buffer));

		/*// Speed bounds, to avoid setting to high speeds to the motor
		if (leftwheel > 1000) {leftwheel = 1000;}
		if (rightwheel > 1000) {rightwheel = 1000;}
		if (leftwheel < -1000) {leftwheel = -1000;}
		if (rightwheel < -1000) {rightwheel = -1000;}
		e_set_speed_left(leftwheel);
		e_set_speed_right(rightwheel);*/

		// Indicate with leds on which side we are turning (leds are great for debugging) 
		if (left_speed>right_speed) {
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
