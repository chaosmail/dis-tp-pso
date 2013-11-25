#
# Standard Makefile for a Webots controller
#
# Author: Olivier.Michel@cyberbotics.com
# Date:   October 12th, 1999.
#

CC = gcc
RM = rm -f *.o *~ $(NAME) pso_chain_sup

ifndef WEBOTS_HOME
    WEBOTS_HOME=/usr/local/webots
endif

ifeq ($(OS),Windows_NT)
	# Windows Makefile
	space :=
	space +=
    WEBOTS_HOME=C:\Program Files (x86)\Webots
    WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
    RM = DEL /F *.o && DEL /F *.exe
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
    	# Linux Makefile
    	WEBOTS_HOME_PATH=/usr/local/webots    	
    endif
    ifeq ($(UNAME_S),Darwin)
        # MacOSX Makefile
        WEBOTS_HOME_PATH=/usr/local/webots
    endif
endif

LIBRARY = -L$(WEBOTS_HOME_PATH)/lib -lController
INCLUDE = -I$(WEBOTS_HOME_PATH)/include/controller/c/

pso_chain_sup:	pso.o pso_chain_sup.c
	$(CC) -o pso_chain_sup pso_chain_sup.c pso.o $(LIBRARY) -lm $(INCLUDE)

pso.o:	pso.c pso.h
	$(CC) -c pso.c

clean:	
	$(RM)
