#
# Standard Makefile for a Webots controller
#
# Author: Olivier.Michel@cyberbotics.com
# Date:   October 12th, 1999.
#

CC = gcc
WEBOTS_HOME=/usr/local/webots

ifeq ($(OS),Windows_NT)
    WEBOTS_HOME=/usr/local/webots
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        WEBOTS_HOME=/usr/local/webots
    endif
    ifeq ($(UNAME_S),Darwin)
        WEBOTS_HOME=/usr/local/webots
    endif
endif

LIBRARY = -L$(WEBOTS_HOME)/lib -lController
INCLUDE = -I$(WEBOTS_HOME)/include/controller/c/

follower:	follower.c
	$(CC) -o follower follower.c $(LIBRARY) $(INCLUDE) -lm

clean:	
	-rm -f *.o *~ $(NAME) follower
