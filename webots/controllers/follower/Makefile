#
# Standard Makefile for a Webots controller
#
# Author: Olivier.Michel@cyberbotics.com
# Date:   October 12th, 1999.
#

CC = gcc
WEBOTS_HOME=/usr/local/webots
LIBRARY = -L$(WEBOTS_HOME)/lib -lController
INCLUDE = -I$(WEBOTS_HOME)/include/controller/c/

follower:	follower.c
	$(CC) -o follower follower.c $(LIBRARY) $(INCLUDE)

clean:	
	-rm -f *.o *~ $(NAME) follower
