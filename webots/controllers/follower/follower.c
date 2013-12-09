#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>


/********** Constants **********/

#define MAX_SPEED 1000.0 // Maximum speed of wheels in each direction
#define MAX_ACC 1000.0 // Maximum amount speed can change in 128 ms
#define NB_SENSOR 8 // Number of proximity sensors

#define DATASIZE 2*NB_SENSOR+6 // Number of elements in particle

// Fitness definitions
#define MAX_DIFF (2*MAX_SPEED) // Maximum difference between wheel speeds
#define MAX_SENS 4096.0 // Maximum sensor value


/********** Global vars **********/

WbDeviceTag ds[NB_SENSOR]; // Webots Device: Sensors
WbDeviceTag emitter; // Webots Device: Emitter of the messages
WbDeviceTag rec; // Webots Device: Handle for the receiver of particles
int braiten;
double good_w[DATASIZE] = {-11.15, -16.93, -8.20, -18.11, -17.99, 8.55, -8.89, 3.52, 29.74,
                           -7.48, 5.61, 11.16, -9.54, 4.58, 1.41, 2.09, 26.50, 23.11,
                           -3.44, -3.78, 23.20, 8.41};


/********** Function declarations **********/

double fitfunc(double[],int);



/********** Function implementations **********/


void reset(void) {
    
    char text[4];
    int i;
    text[1]='s';
    text[3]='\0';
    
    for (i=0;i<NB_SENSOR;i++) {
        text[0]='p';
        text[2]='0'+i;
        ds[i] = wb_robot_get_device(text); // distance sensors
    }
    
    emitter = wb_robot_get_device("emitter");
    rec = wb_robot_get_device("receiver");
}

int main() {

    double buffer[255];
    double *rbufferPointer;
    double rbuffer[DATASIZE+1];
    double fit;
    int i, j;

    wb_robot_init();
    reset();
    for(i=0;i<NB_SENSOR;i++) {
        distance_sensor_enable(ds[i],64);
    }
    
    receiver_enable(rec,32);
    
    differential_wheels_enable_encoders(64);
    braiten = 0; // Don't run forever
    robot_step(64);
    
    while (1) {
        
        // Wait for data: Weights for Braitenberg Controller
        while (receiver_get_queue_length(rec) == 0) {
            robot_step(64);
        }

        // Read as long as data is available
        while (receiver_get_queue_length(rec) > 0) {

            rbufferPointer = (double *)wb_receiver_get_data(rec);
            
            for (j=0; j<DATASIZE+1; j++)
                rbuffer[j] = rbufferPointer[j];
            
             printf("*received weights: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", rbufferPointer[0], rbufferPointer[1], rbufferPointer[2], rbufferPointer[3], rbufferPointer[4], rbufferPointer[5], rbufferPointer[6], rbufferPointer[7], rbufferPointer[DATASIZE]);
            wb_receiver_next_packet(rec);
        }

        // printf("received weights: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", rbuffer[0], rbuffer[1], rbuffer[2], rbuffer[3], rbuffer[4], rbuffer[5], rbuffer[6], rbuffer[7], rbuffer[DATASIZE]);

        // Check for pre-programmed avoidance behavior
        // We need this in the epuck when PSO is done
        if (rbuffer[DATASIZE] == -1.0) {

            braiten = 1;
            fitfunc(good_w,100);
        }
        // Otherwise, run provided controller
        else {

            // printf("Start evaluating fitness\n");
            fit = fitfunc(rbuffer,rbuffer[DATASIZE]); //evaluates fitness of the received particle
            // printf("Stopped evaluating fitness\n");
            
            buffer[0] = fit;
            wb_emitter_send(emitter,(void *)buffer,sizeof(double)); //sends the fitness back to the controller.
        }
    }

    return 0;
}

// Generate random number from 0 to 1
double rnd() {
    return (double)rand()/RAND_MAX;
}

// Generate Gaussian random number with 0 mean and 1 std
double gauss(void) {
    double x1, x2, w;

    do {
        x1 = 2.0 * rnd() - 1.0;
        x2 = 2.0 * rnd() - 1.0;
        w = x1*x1 + x2*x2;
    } while (w >= 1.0);

    w = sqrt((-2.0 * log(w))/w);
    return(x1*w);
}

// S-function to transform v variable to [0,1]
double s(double v) {
    if (v > 5)
        return 1.0;
    else if (v < -5)
        return 0.0;
    else
        return 1.0/(1.0 + exp(-1*v));
}

// Find the fitness for obstacle avoidance of the passed controller
double fitfunc(double weights[DATASIZE], int its) {
    
    double left_speed,right_speed; // Wheel speeds
    double old_left, old_right; // Previous wheel speeds (for recursion)
    int left_encoder,right_encoder;
    double ds_value[NB_SENSOR];
    int i,j; // Loop vars

    // Print the received weights
    /*printf("%d iterations with ",its);
    for (i=0; i<DATASIZE; i++) {
        printf(" %.2f",weights[DATASIZE]);
    }
    printf("\n");*/

    // Fitness variables
    /*double fit_speed;           // Speed aspect of fitness
    double fit_diff;            // Speed difference between wheels aspect of fitness
    double fit_sens;            // Proximity sensing aspect of fitness*/
    double fit_range;           //range aspect of fitness
    double fit_bearing;          //bearing aspect of fitness
    double fit_relative_heading; //relative heading aspect of fitness
    double new_leader_range, new_leader_bearing, new_relative_heading; // received leader range and bearing and relative heading
    double *rbbuffer;                  // buffer for the range and bearin
    double sens_val[NB_SENSOR]; // Average values for each proximity sensor
    double fitness;             // Fitness of controller

    // Initially no fitness measurements
    /*fit_speed = 0.0;
    fit_diff = 0.0;*/
    fit_range=0.0;
    fit_bearing=0.0;
    fit_relative_heading=0.0;

    for (i=0;i<NB_SENSOR;i++) {
        sens_val[i] = 0.0;
    }
    //fit_sens = 0.0;
    old_left = 0.0;
    old_right = 0.0;

    // Evaluate fitness repeatedly
    for (j=0;j<its;j++) {
        
        if (braiten) j--;            // Loop forever

        ds_value[0] = (double) wb_distance_sensor_get_value(ds[0]);
        ds_value[1] = (double) wb_distance_sensor_get_value(ds[1]);
        ds_value[2] = (double) wb_distance_sensor_get_value(ds[2]);
        ds_value[3] = (double) wb_distance_sensor_get_value(ds[3]);
        ds_value[4] = (double) wb_distance_sensor_get_value(ds[4]);
        ds_value[5] = (double) wb_distance_sensor_get_value(ds[5]);
        ds_value[6] = (double) wb_distance_sensor_get_value(ds[6]);
        ds_value[7] = (double) wb_distance_sensor_get_value(ds[7]);

        // Weights for the follower controller
        // printf("my weights: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", weights[0], weights[1], weights[2], weights[3], weights[4], weights[5], weights[6], weights[7]);

        // Feed proximity sensor values to neural net
        left_speed = 0.0;
        right_speed = 0.0;
        /*for (i=0;i<NB_SENSOR;i++) {
            left_speed += weights[i]*ds_value[i];
            right_speed += weights[i+NB_SENSOR+1]*ds_value[i];
        }*/
        
        //only use sensors 0,1,6,7 and use symmetry
        left_speed = weights[0]*ds_value[0]+weights[1]*ds_value[1]+weights[6]*ds_value[6]+weights[7]*ds_value[7];
        right_speed = weights[0]*ds_value[7]+weights[1]*ds_value[6]+weights[6]*ds_value[1]+weights[7]*ds_value[0];
        
        
        left_speed /= 200.0;
        right_speed /= 200.0;

        /*// Add the recursive connections
        left_speed += weights[2*NB_SENSOR+2]*(old_left+MAX_SPEED)/(2*MAX_SPEED);
        left_speed += weights[2*NB_SENSOR+3]*(old_right+MAX_SPEED)/(2*MAX_SPEED);
        right_speed += weights[2*NB_SENSOR+4]*(old_left+MAX_SPEED)/(2*MAX_SPEED);
        right_speed += weights[2*NB_SENSOR+5]*(old_right+MAX_SPEED)/(2*MAX_SPEED);
        // Add neural thresholds
        left_speed += weights[NB_SENSOR];
        right_speed += weights[2*NB_SENSOR+1];*/
        // Apply neuron transform
        left_speed = MAX_SPEED*(2.0*s(left_speed)-1.0);
        right_speed = MAX_SPEED*(2.0*s(right_speed)-1.0);

        // Make sure we don't accelerate too fast
        if (left_speed - old_left > MAX_ACC) left_speed = old_left+MAX_ACC;
        if (left_speed - old_left < -MAX_ACC) left_speed = old_left-MAX_ACC;
        if (right_speed - old_right > MAX_ACC) left_speed = old_right+MAX_ACC;
        if (right_speed - old_right < -MAX_ACC) left_speed = old_right-MAX_ACC;
        
        // Make sure speeds are within bounds
        if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
        if (left_speed < -1.0*MAX_SPEED) left_speed = -1.0*MAX_SPEED;
        if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
        if (right_speed < -1.0*MAX_SPEED) right_speed = -1.0*MAX_SPEED;

        // Set new old speeds
        old_left = left_speed;
        old_right = right_speed;

        left_encoder = wb_differential_wheels_get_left_encoder();
        right_encoder = wb_differential_wheels_get_right_encoder();
        if (left_encoder>9000) wb_differential_wheels_set_encoders(0,right_encoder);
        if (right_encoder>1000) wb_differential_wheels_set_encoders(left_encoder,0);
        
        // Set the motor speeds
        wb_differential_wheels_set_speed((int)left_speed,(int)right_speed);
        robot_step(128); // run one step

        // Get current fitness value

        /*// Average speed
        fit_speed += (fabs(left_speed) + fabs(right_speed))/(2.0*MAX_SPEED);
        // Difference in speed
        fit_diff += fabs(left_speed - right_speed)/MAX_DIFF;
        // Sensor values
        for (i=0;i<NB_SENSOR;i++) {
            sens_val[i] += ds_value[i]/MAX_SENS;*/

        /* Receive leader range, bearing and relative heading of leader */
        while (wb_receiver_get_queue_length(rec) > 0) {
            
            rbbuffer = (double *)wb_receiver_get_data(rec);

            // this data is received
            // printf("x %.2f, z %.2f, phi %.2f\n",rbbuffer[0] , rbbuffer[1], rbbuffer[2]);

            new_leader_range = sqrt(rbbuffer[0]*rbbuffer[0] + rbbuffer[1]*rbbuffer[1]);
            new_leader_bearing = -atan2(rbbuffer[0],rbbuffer[1]);
            new_relative_heading = rbbuffer[2];

            wb_receiver_next_packet(rec);
        }

        fit_range += new_leader_range;
        fit_bearing += new_leader_bearing;
        fit_relative_heading += new_relative_heading;
    }

    /*// Find most active sensor
    for (i=0;i<NB_SENSOR;i++) {
        if (sens_val[i] > fit_sens) fit_sens = sens_val[i];
    }*/
    // Average values over all steps
    /*fit_speed /= its;
    fit_diff /= its;
    fit_sens /= its;*/
    fit_range /= its;
    fit_bearing /= its;
    fit_relative_heading /= its;

    // Better fitness should be higher
    int A=1; //importance coefficient of range
    int B=1; //importance coefficient of bearing
    int C=1; //importance coefficient of relative heading
    
    // What about negative fitness?
    // shouldnt we calculate just positive ones?
    double tmpVal = A*fit_range + B*fabs(fit_bearing) + C*fabs(fit_relative_heading);

    if (tmpVal == 0) {
        fitness = 100;
        printf("All zero\n");
    }
    else {
        fitness = 1/(A*fit_range+B*fabs(fit_bearing)+C*fabs(fit_relative_heading));
    }

    //printf("fitness: %.2f \n", fitness);
    //printf("fitness %.2f = range %.2f, bearing %.2f, heading %.2f\n", fitness, fit_range, fit_bearing, fit_relative_heading);
    return fitness;
}

