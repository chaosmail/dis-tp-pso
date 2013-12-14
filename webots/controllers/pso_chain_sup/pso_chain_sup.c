#include <stdio.h>
#include <float.h>
#include <math.h>
#include <unistd.h>
#include "pso.h"
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>


/********** Constants **********/

/* Save Results */
#define SAVE_RESULTS 1
#define RESULTS_DIR "results"

#define ROBOTS 3                        
#define MAX_ROB 3
#define ROB_RAD 0.035
#define ARENA_SIZE 1.89

#define NB_ALL_SENSOR 8 // Number of all proximity sensors
#define NB_SENSOR 6 // Number of used proximity sensors

/* PSO definitions */
#define SWARMSIZE 10                    // Number of particles in swarm
#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 20.0                       // Maximum velocity particle can attain
#define MININIT -20.0                   // Lower bound on initialization value
#define MAXINIT 20.0                    // Upper bound on initialization value
#define PSO_ITS 60                     // Number of iterations for PSO to run
#define DATASIZE NB_SENSOR+3            // Number of elements in particle

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_ITS 65                      // Number of fitness steps to run during evolution

#define FINALRUNS 10
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8


/********** Global vars **********/

// Needed for localisation
// static WbNodeRef robs[ROBOTS];
static WbNodeRef robs[ROBOTS+1]; 
static WbFieldRef robs_translation[ROBOTS+1];
static WbFieldRef robs_rotation[ROBOTS+1];
double loc[ROBOTS+1][4]; //needed to communicate leader positions to follower

WbDeviceTag emitter[MAX_ROB];
WbDeviceTag leader_emitter;
WbDeviceTag rec[MAX_ROB];
const double *locTemp[ROBOTS+1];
const double *rot[ROBOTS+1];
double new_loc[ROBOTS+1][3];
double new_rot[ROBOTS+1][4];

// Initial Weights
// Use -DBL_MAX to be randomly generated in PSO
// double initial_weight[DATASIZE] = { -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX };
double initial_weight[DATASIZE] = { 5.97, 0.16, 2.05, -2.91, -3.11, 95.31, 68.25, -87.65, 10.36 };

// Velocity of Changement of Weights (Particle velocity)
// Use -DBL_MAX to be randomly generated in PSO
double pso_velocity[DATASIZE] = { -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX};
// double pso_velocity[DATASIZE] = {5,5,5,5,5,5,5,5,5};

/********** Function declarations **********/

void calc_fitness(double[][DATASIZE],double[],int,int);
//void random_pos(int);
void reset_pos(int);
void nRandom(int[][SWARMSIZE],int);
void nClosest(int[][SWARMSIZE],int);
void fixedRadius(int[][SWARMSIZE],double);
void writeWeightsToFile(double, double[DATASIZE]);

/********** Function implementations **********/

void reset(void) {

    // Check for the maximal number of robots
    if (ROBOTS > 9) {
        printf("Maximal number of follower robots is 9");
        return;
    }

    char rob[] = "rob0";
    char em[] = "emitter0";
    char receive[] = "receiver0";

    int i; // Loop var

    //emmiters & receivers -> not for leader
    for (i=0;i<ROBOTS;i++) {

        emitter[i] = wb_robot_get_device(em);

        if (emitter[i]==0) {
            printf("missing emitter %d\n",i);
        }

        rec[i] = wb_robot_get_device(receive);

        em[7]++; // change name 'emitter.'
        receive[8]++; // change name 'receiver.'
    }
    
    /*enable emitter for leader*/
    leader_emitter=wb_robot_get_device("emitter3");

    //Position -> for all robots
    for (i=0; i<=ROBOTS; i++) {

        robs[i] = wb_supervisor_node_get_from_def(rob);
        robs_translation[i] = wb_supervisor_node_get_field(robs[i],"translation");
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
        locTemp[i] = wb_supervisor_field_get_sf_vec3f(robs_translation[i]);

        new_loc[i][0] = locTemp[i][0]; new_loc[i][1] = locTemp[i][1]; new_loc[i][2] = locTemp[i][2];
        rot[i] = wb_supervisor_field_get_sf_rotation(robs_rotation[i]);
        new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];

        rob[3]++; // change name 'rob.'
    }
}

int main() {

    printf("*** Started PSO ***\n");
    
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL)
        fprintf(stdout, "Current working dir: %s\n", cwd);

    writeWeightsToFile(0,initial_weight);

    double *weights; // Evolved result
    double buffer[255]; // sending buffer
    int i,j,k; // Loop vars

    double fit, endfit, fitvals[FINALRUNS], w[MAX_ROB][DATASIZE], f[MAX_ROB];
    double bestfit, bestw[DATASIZE];

    wb_robot_init();
    reset();

    for (i=0;i<MAX_ROB;i++) {
        wb_receiver_enable(rec[i],32);
    }

    wb_robot_step(256);

    // Evolve controllers
    endfit = 0.0;
    bestfit = 0.0;

    for (j=0; j<100000; j++) {

        /* Get result of evolution */
        weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,PSO_ITS,DATASIZE,ROBOTS,initial_weight,pso_velocity);

        /* Calculate performance */
        fit = 0.0;
        for (i=0;i<MAX_ROB;i++) {

            for (k=0;k<DATASIZE;k++) {

                w[i][k] = weights[k];
            }
        }

        /* Run FINALRUN tests and calculate average */
        for (i=0;i<FINALRUNS;i+=MAX_ROB) {

            calc_fitness(w,f,FIT_ITS,MAX_ROB);

            for (k=0;k<MAX_ROB && i+k<FINALRUNS;k++) {

                // printf("Fitness: %f\n",f[k]);

                fitvals[i+k] = f[k];
                fit += f[k];
            }
        }
        fit /= FINALRUNS;

        /* Check for new best fitness */
        if (fit > bestfit) {

            bestfit = fit;

            for (i = 0; i < DATASIZE; i++) {
                bestw[i] = weights[i];
                initial_weight[i] = weights[i]; // to send the best weights back into the next pso
            }

            writeWeightsToFile(fit,weights);
        }

        printf("Performance: %.3f\n",fit);
        endfit += fit/10;
    }
    printf("Average performance: %.3f\n",endfit);

    /* Send best controller to robots */
    for (j=0;j<DATASIZE;j++) {
        buffer[j] = bestw[j];
    }

    buffer[DATASIZE] = 1000000;

    // Print out the best results
    printf("***Best weights:\n");

    for (j=0;j<DATASIZE;j++) {
        if (j == DATASIZE-1)
            printf("%.2f };\n",bestw[j]);
        else if (j == 0)
            printf("double initial_weight[DATASIZE] = { %.2f, ",bestw[j]);
        else
            printf("%.2f, ",bestw[j]);
    }
    
    for (i=0;i<ROBOTS;i++) {
        wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE+1)*sizeof(double));
    }

    /* Wait forever */
    while (1) wb_robot_step(64);

    return 0;
}

void writeWeightsToFile(double performance, double weights[DATASIZE]) {

    int j;

    char fileName[256];
    snprintf(fileName, sizeof fileName, "../../%s/%.4f_performance.txt", RESULTS_DIR, performance);

    FILE *f = fopen(fileName, "w");
    if (f == NULL)
    {
        printf("Error opening file %s!\n", fileName);
        exit(-1);
    }

    for (j=0;j<DATASIZE;j++) {
        if (j == DATASIZE-1)
            fprintf(f,"%.2f };\n",weights[j]);
        else if (j == 0)
            fprintf(f,"double initial_weight[DATASIZE] = { %.2f, ",weights[j]);
        else
            fprintf(f,"%.2f, ",weights[j]);
    }

    fclose(f);
}

// Makes sure no robots are overlapping
char valid_locs(int rob_id) {
    int i;

    for (i = 0; i < MAX_ROB; i++) {
        if (rob_id == i) continue;
        if (pow(new_loc[i][0]-new_loc[rob_id][0],2) +
                pow(new_loc[i][2]-new_loc[rob_id][2],2) < (2*ROB_RAD+0.01)*(2*ROB_RAD+0.01))
            return 0;
    }
    return 1;
}

// Randomly position specified robot
/*void random_pos(int rob_id) {
  //printf("Setting random position for %d\n",rob_id);
  new_rot[rob_id][0] = 0.0;
  new_rot[rob_id][1] = 1.0;
  new_rot[rob_id][2] = 0.0;
  new_rot[rob_id][3] = 2.0*3.14159*rnd();
  
  do {
    new_loc[rob_id][0] = ARENA_SIZE*rnd() - ARENA_SIZE/2.0;
    new_loc[rob_id][2] = ARENA_SIZE*rnd() - ARENA_SIZE/2.0;
    //printf("%d at %.2f, %.2f\n", rob_id, new_loc[rob_id][0], new_loc[rob_id][2]);
  } while (!valid_locs(rob_id));

  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[rob_id],"translation"), new_loc[rob_id]);
  wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[rob_id],"rotation"), new_rot[rob_id]);
}*/
void reset_pos(int rob_id) {

    new_rot[rob_id][0] = 0.0;
    new_rot[rob_id][1] = 1.0;
    new_rot[rob_id][2] = 0.0;
    new_rot[rob_id][3] = -1.57;

    new_loc[rob_id][0]=-0.3+0.1*rob_id;
    new_loc[rob_id][2]=0;
    
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[rob_id],"translation"), new_loc[rob_id]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[rob_id],"rotation"), new_rot[rob_id]);
}

// Distribute fitness functions among robots
void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs) {

    double buffer[DATASIZE+1];// to send particles to robots
    double buffer_loc[255];//to send positions to robot
    double buffer_leader[1];//to tell leader to reset
    double *rbuffer; //to get fitness from robots
    double global_x,global_z,rel_x,rel_z; //for localisation
    int i,j;
    int cnt = 1;
    int send_interval = 10;

    // printf("Iterations: %d\n", its);

    // for(k=0;k<ROBOTS;k++){  //WATCH OUT this sends the same weight k to all the robots->homogenous

        //send weights to followers 0,1,2
        for (i=0;i<numRobs;i++) {

            // reset fitness value
            fit[i]=0;

            //put robot back to initial chain position
            reset_pos(i);
            //printf("reset");
            

            if (i==2){
                //resets also leader
                reset_pos(3);
                buffer_leader[0] = 1.0;
                wb_emitter_send(leader_emitter,(char *)buffer_leader,1*sizeof(double));
            }

            for (j=0;j<DATASIZE;j++) {
                buffer[j] = weights[i][j];
            }

            buffer[DATASIZE] = (double) its;

            // printf("Use these weights: %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", buffer[0], buffer[1], buffer[2] ,buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[DATASIZE]);

            wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE+1)*sizeof(double));
            //printf("%f %f %f %f %f\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
        }

        // Wait for response */
        while (wb_receiver_get_queue_length(rec[0]) == 0){

            for (i=0;i<ROBOTS;i++) {

                //leader is the one in front
                loc[i+1][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[i+1])[0];
                loc[i+1][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[i+1])[1];
                loc[i+1][2] = wb_supervisor_field_get_sf_vec3f(robs_translation[i+1])[2];
                loc[i+1][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[i+1])[3];


                loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0];
                loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[1];
                loc[i][2] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2];
                loc[i][3] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3];
                
                //printf("Robot %i heading: %.2f\n", i , loc[i][3]);
                //printf("Robot %i y: %.2f", i , loc[i][1]);
                
                // Find global relative coordinates
                global_x = loc[i][0] - loc[i+1][0];
                global_z = loc[i][2] - loc[i+1][2];

                // Calculate relative coordinates
                rel_x = -global_x*cos(loc[i][3]) + global_z*sin(loc[i][3]);
                rel_z = global_x*sin(loc[i][3]) + global_z*cos(loc[i][3]);

                buffer_loc[0] = rel_x; // distance in direction of the heading of the robot
                buffer_loc[1] = rel_z;// distance perpendicular to heading
                buffer_loc[2] = loc[i+1][3] - loc[i][3]; // relative heading
                while (buffer_loc[2] > M_PI) buffer_loc[2] -= 2.0*M_PI;
                while (buffer_loc[2] < -M_PI) buffer_loc[2] += 2.0*M_PI;

                // dont send each loop
                if (cnt%send_interval == 0) {

                    // data is send
                    // printf("Robot %i: x %.2f, z %.2f, phi %.2f\n",i , buffer_loc[0] , buffer_loc[1], buffer_loc[2]);

                    // send relative leaders positions to follower
                    wb_emitter_send(emitter[i],(char *)buffer_loc,3*sizeof(double));
                    
                    // Reset the count var
                    cnt = 0;
                }

                //wait 1 timestep? Does the buffer of the emmited message pile up??

                // Check error in position of robot
                //rel_x = global_x*cos(loc[0][3]) - global_z*sin(loc[0][3]);
                //rel_z = -global_x*sin(loc[0][3]) - global_z*cos(loc[0][3]);
                //temp_err = sqrt(pow(rel_x-good_rp[i][0],2) + pow(rel_z-good_rp[i][1],2));
                //if (print_enabled)
                //  printf("Err %d: %.3f, ",i,temp_err);
                //err += temp_err/ROBOTS;
                cnt++;
            }
            wb_robot_step(64);
        }

        // Get fitness values */
        for (i=0;i<numRobs;i++) {

            rbuffer = (double *)wb_receiver_get_data(rec[i]);
            
            /*if (i==2) {
                fit[i] += rbuffer[0];
                //printf("fit %f",fit[i]);
            }
            else {*/
                fit[i] += rbuffer[0];
            //}

            //printf("received fitness: %0.2f\n",rbuffer[0]);
            wb_receiver_next_packet(rec[i]);
        }

        //fit[k]/=ROBOTS;
    //}
}

// Evolution fitness function
// This function is called in file pso.c in function findPerformance()
void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE]) {
    int i,j;

    // printf("Use these weights: %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", weights[0][0], weights[0][1], weights[0][2] ,weights[0][3], weights[0][4], weights[0][5], weights[0][6], weights[0][7]);

    calc_fitness(weights,fit,FIT_ITS,ROBOTS);

#if NEIGHBORHOOD == RAND_NB
    nRandom(neighbors,2*NB);
#endif

#if NEIGHBORHOOD == NCLOSE_NB
    nClosest(neighbors,2*NB);
#endif

#if NEIGHBORHOOD == FIXEDRAD_NB
    fixedRadius(neighbors,RADIUS);
#endif
}

/* Get distance between robots */
double robdist(int i, int j) {
    return sqrt(pow(loc[i][0]-loc[j][0],2) + pow(loc[i][2]-loc[j][2],2));
}

/* Choose n random neighbors */
void nRandom(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

    int i,j;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Clear old neighbors */
        for (j = 0; j < ROBOTS; j++)
            neighbors[i][j] = 0;

        /* Set new neighbors randomly */
        for (j = 0; j < numNB; j++)
            neighbors[i][(int)(SWARMSIZE*rnd())] = 1;

    }
}

/* Choose the n closest robots */
void nClosest(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

    int r[numNB];
    int tempRob;
    double dist[numNB];
    double tempDist;
    int i,j,k;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Clear neighbors */
        for (j = 0; j < numNB; j++)
            dist[j] = ARENA_SIZE;

        /* Find closest robots */
        for (j = 0; j < ROBOTS; j++) {

            /* Don't use self */
            if (i == j) continue;

            /* Check if smaller distance */
            if (dist[numNB-1] > robdist(i,j)) {

                dist[numNB-1] = robdist(i,j);
                r[numNB-1] = j;

                /* Move new distance to proper place */
                for (k = numNB-1; k > 0 && dist[k-1] > dist[k]; k--) {

                    tempDist = dist[k];
                    dist[k] = dist[k-1];
                    dist[k-1] = tempDist;
                    tempRob = r[k];
                    r[k] = r[k-1];
                    r[k-1] = tempRob;
                }
            }
        }

        /* Update neighbor table */
        for (j = 0; j < ROBOTS; j++)
            neighbors[i][j] = 0;
        for (j = 0; j < numNB; j++)
            neighbors[i][r[j]] = 1;

    }

}

/* Choose all robots within some range */
void fixedRadius(int neighbors[SWARMSIZE][SWARMSIZE], double radius) {

    int i,j;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Find robots within range */
        for (j = 0; j < ROBOTS; j++) {

            if (i == j) continue;

            if (robdist(i,j) < radius) neighbors[i][j] = 1;
            else neighbors[i][j] = 0;

        }
    }
}

void step_rob() {
    wb_robot_step(64);
}
