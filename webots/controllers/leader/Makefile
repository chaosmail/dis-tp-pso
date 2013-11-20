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

leader:	leader.c
	$(CC) -o leader leader.c $(LIBRARY) $(INCLUDE) -lm

clean:	
	-rm -f *.o *~ $(NAME) leader
