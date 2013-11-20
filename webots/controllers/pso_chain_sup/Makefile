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

pso_chain_sup:	pso.o pso_chain_sup.c
	$(CC) -o pso_chain_sup pso_chain_sup.c pso.o $(LIBRARY) -lm $(INCLUDE)

pso.o:	pso.c pso.h
	$(CC) -c pso.c

clean:	
	-rm -f *.o *~ $(NAME) pso_chain_sup
