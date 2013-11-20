# Project: Evolving a “following and chain-building” behavior using PSO

Using PSO, optimize a robot-following and chain-creation behavior for the real e-puck
platform. The goal is to create a long chain of robots. First, in Webots, develop a
Braitenberg type algorithm and implement your learning strategy. Then, implement
your optimized behavior on the real platform using 4 e-puck robots (1 leader, 3
followers). Discuss the strengths and weaknesses of this approach and identify possible
changes to the heuristics which might improve the transition from the simulated e-puck
to the real one.

## Topics and Goals

### Prequisits and Design

* design PSO
* design fitness function
* design neural network controller (braitenberg controller)
* do nonlinear sensor calibration with epuck

### Implementation Webots

* setup environement and trajectory
* implement fitness function
* implement PSO

### Optimization in Webots

* run PSO
* run PSO with noise

### Implementation Epuck

* implement lookup table for sensor values
* implement braitenberg values

### Optional Optimization

* noiseresistant PSO
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

### Common Errors 

#### Ubuntu 13.10 x64

While building the PSO controllers, libjpeg is missing
```
/usr/bin/ld: warning: libjpeg.so.62, needed by /usr/local/webots/lib/libController.so, not found (try using -rpath or -rpath-link)
```
Solution: Install libjpeg62
```
sudo apt-get install libjpeg62
```

While running Webots 7.3.0 x64, libraw5 is missing (but libraw9 is installed)
```
/usr/local/webots7/webots-bin: error while loading shared libraries: libraw.so.5: cannot open shared object file: No such file or directory
```
Solution: Link the libraw9 lib to libraw5
```
cd /usr/lib
sudo ln -s /usr/lib/libraw.so.5 /usr/lib/libraw.so.9
```

## Useful Links

### Software and Coding

* [C++ Coding Standard](http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml) for useful coding conventions
* [Git Documentation](http://git-scm.com/documentation) for source code version control
* [Latex Help](http://en.wikibooks.org/wiki/LaTeX) for writing of scientific papers
* [Markdown Cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet) for documentation on Github
