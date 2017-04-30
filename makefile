# 046267 Computer Architecture - Spring 2016 - HW #2
# makefile for test environment

all: bp_main

# Environment for C 
CC = gcc
CFLAGS = -std=c99 -Wall

# Environment for C++ 
CXX = g++
CXXFLAGS = -std=c++11 -Wall

# Automatically detect whether the bp is C or C++
# Must have either bp.c or bp.cpp - NOT both
SRC_BP = $(wildcard bp.c bp.cpp)
SRC_GIVEN = bp_main.c
EXTRA_DEPS = bp_api.h

OBJ_GIVEN = $(patsubst %.c,%.o,$(SRC_GIVEN))
OBJ_BP = bp.o
OBJ = $(OBJ_GIVEN) $(OBJ_BP)

#$(info OBJ=$(OBJ))


ifeq ($(SRC_BP),bp.c)
bp_main: $(OBJ)
	$(CC) -o $@ $(OBJ)

bp.o: bp.c
	$(CC) -c $(CFLAGS) -o $@ $^

else
bp_main: $(OBJ)
	$(CXX) -o $@ $(OBJ)

bp.o: bp.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $^
endif

$(OBJ_GIVEN): %.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $^


.PHONY: clean
clean:
	rm -f bp_main $(OBJ)
