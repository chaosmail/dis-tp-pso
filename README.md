# Evolving a “following and chain-building” behavior using PSO

## Overview

Using PSO, optimize a robot-following and chain-creation behavior for the real e-puck
platform. The goal is to create a long chain of robots. First, in Webots, develop a
Braitenberg type algorithm and implement your learning strategy. Then, implement
your optimized behavior on the real platform using 4 e-puck robots (1 leader, 3
followers). Discuss the strengths and weaknesses of this approach and identify possible
changes to the heuristics which might improve the transition from the simulated e-puck
to the real one.

## Design

### PSO

* Individual Performance
* Group Solution
* Heterogeneous Approach

### Fitness Function

* minimize relative range
* minimize relative bearing
* minimize relative orientation
* penalize reaching MAX_SPEED

## Tasks

### Prequisits and Design

* ~~design PSO~~
* ~~design fitness function~~
* ~~design neural network controller (braitenberg controller)~~
* ~~do nonlinear sensor calibration with epuck~~
* ~~fit a curve on the sensor calibration values~~

### Implementation Webots

#### Environement

* ~~Setup a leader~~
* ~~Setup 3 followers~~
* ~~Setup a PSO superviser~~

#### Leader

* ~~Do a random trajectory~~
* ~~follow a predefined trajectory~~

#### Superviser

* ~~read coordinates of each personal leader and follower and sends that values to each follower (see LAB05 odometry flock_super.c)~~ @Etienne
* ~~send the coordinates of the personal leader in hte followers coordinate system to the follower~~ @Etienne

##### pso

* ~~Send particles to robot (see LAB06 PSO pso_obs_sup.c)~~ @Christoph
* ~~Receive all the fitness values (see LAB06 PSO pso_obs_sup.c)~~ @Christoph
* ~~get best local performance, best neighborhood performance (see LAB06 PSO pso_obs_sup.c)~~ @Christoph

#### Follower

##### main function

* ~~get data for braitenberg controller (see LAB06 PSO pso_obs.c)~~
* ~~move (see LAB06 PSO pso_obs.c)~~
* ~~calc fitness and send back to superviser (see LAB06 PSO pso_obs.c)~~

##### fitness function

* ~~Calculate realtive range (see LAB05 odometry follower3.c)~~ @Etienne
* ~~Calculate relative bearing (see LAB05  odometry follower3.c)~~ @Etienne
* ~~Calculate realtive orientation (see LAB05 odometry follower3.c)~~ @Etienne
* ~~Move robot (see LAB06 PSO obs_con.c)~~
* ~~Calculate the fitness (see LAB06 PSO obs_con.c)~~

##### wheel speed 

* ~~Implement Braitenberg controller (see LAB01)~~ @Alice

### Optimization in Webots

* ~~run PSO~~
* ~~run PSO with noise~~
* ~~optimize trajectory~~
* optimize PSO parameters such as neighborhood, swarmsize, personal best weight, neighborhoods best weight, velocities, etc.
* optimize fitness function

### Implementation Epuck

* implement lookup table for sensor values
* implement braitenberg controller and weights (see LAB04 obstacleavoidance main.c)

### Optional Optimization

* ~~noiseresistant PSO~~
* PID controller

## Development

### Getting started

Request a free Webots Licence with your EPFL email adress from www.cyberbotics.com/registration/webots/trial/

Install Webots 6.2.4 64-bit (Linux)
```
wget http://www.cyberbotics.com/archive/linux/webots-6.2.4-x86-64.tar.bz2
tar jxf webots-6.2.4-x86-64.tar.bz2
sudo mv webots /usr/local/webots
sudo ln -s /usr/local/webots/webots /usr/bin/webots
```

Clone the git repository
```
git clone https://github.com/chaosmail/dis-tp-pso.git
```

Build the controllers
```
cd dis-tp-pso/webots/controllers
make
```

Run Webots
```
webots &
```

Open the World dis-tp-pso/webots/worlds/pso_project.wbt and get started!

## Useful Links

### Software and Coding

* [C++ Coding Standard](http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml) for useful coding conventions
* [Git Documentation](http://git-scm.com/documentation) for source code version control
* [Latex Help](http://en.wikibooks.org/wiki/LaTeX) for writing of scientific papers
* [Markdown Cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet) for documentation on Github
