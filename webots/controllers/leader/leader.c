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
#define NB_ALL_SENSOR 8 // Number of all proximity sensors
#define NB_SENSOR 6 // Number of used proximity sensors

#define DATASIZE NB_SENSOR+3 // Number of elements in particle

// Fitness definitions
#define MAX_DIFF (2*MAX_SPEED) // Maximum difference between wheel speeds
#define MAX_SENS 4096.0 // Maximum sensor value


/********** Global vars **********/

WbDeviceTag ds[NB_ALL_SENSOR]; // Webots Device: Sensors
WbDeviceTag emitter; // Webots Device: Emitter of the messages
WbDeviceTag rec; // Webots Device: Handle for the receiver of particles
int braiten;


/********** Function declarations **********/

//double fitfunc(double[],int);
double rnd();
double gauss();
double unf(double, double);


/********** Function implementations **********/

void reset(void) {
    char text[4];
    int i;
    text[1]='s';
    text[3]='\0';
    for (i=0;i<NB_ALL_SENSOR;i++) {
        text[0]='p';
        text[2]='0'+i;
        ds[i] = wb_robot_get_device(text); // distance sensors
    }
    emitter = wb_robot_get_device("emitter");
    rec = wb_robot_get_device("receiver");
}

int main() {

    double fit;
    int i;
    double *rbufferPointer;
    double restart=0;

    wb_robot_init();
    reset();
    for(i=0;i<NB_ALL_SENSOR;i++) {
        distance_sensor_enable(ds[i],64);
    }
    receiver_enable(rec,32);
    differential_wheels_enable_encoders(64);
    braiten = 0; // Don't run forever
    
    int speedl = 0;
    int speedr = 0;
    int oldSpeedl = 0;
    int oldSpeedr = 0;
    
    int length = 12;
    //printf("length: %i \n", length);

    int counter = 0;
    
    /*int trajl[12] = {6,6,6,6,
                      6,6,4,4,
                      6,6,6,6};
                     
    int trajr[12] = {6,4,4,6,
                      6,6,6,6,
                      6,6,6,6};*/
    

    robot_step(64);
    while (1) {

        oldSpeedl = speedl;
        oldSpeedr = speedr;

        // Leader just moves ranomly
        // weighted movement
        speedl = (int) ((MAX_SPEED*unf(0.6,1))*7 + oldSpeedl*3)/10;
        speedr = (int) ((MAX_SPEED*unf(0.6,1))*7 + oldSpeedr*3)/10;
        /*speedl = (int) (MAX_SPEED*rand());
        speedr = (int) (MAX_SPEED*rand());*/
        
        while (receiver_get_queue_length(rec) > 0) {
        rbufferPointer = (double *)wb_receiver_get_data(rec);
        restart=rbufferPointer[0];
        //robot_step(64);
        wb_receiver_next_packet(rec);
        }
         if(restart==1.0){
          counter=0;
          restart=0;
        }
        // printf("restart: %f\n",restart);
        // wb_differential_wheels_set_speed(100*trajl[counter%length],100*trajr[counter%length]);
        
        wb_differential_wheels_set_speed(speedl,speedr);
        
        // printf("counter: %i \n", counter);
        
        //counter++;
        robot_step(10*64);
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
