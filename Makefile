# Makefile

CXX = g++
CFLAGS = -g -Wall

LIB = -lode -ldrawstuff -lGL -lGLU -lX11 -lrt -lm -lpthread -lstdc++

LIBDIR = -L/usr/lib/x86_64-linux-gnu -L/usr/local/etc/ode-0.16.2/drawstuff/src/.libs  -L/usr/local/etc/ode-0.16.2/ode/src/.libs -L/usr/lib 
INCDIR = -I/usr/local/etc/ode-0.16.2/include -I/usr/local/etc/ode-0.16.2/ode/src -I/usr/include

OBJS = simulator simulator.o main.o

ALL: main.o simulator.o
	$(CXX) $(CFLAGS) -o main main.o simulator.o $(INCDIR) $(LIBDIR) $(LIB) 

main.o: main.cpp
	$(CXX) $(CFLAGS) -o main.o -c main.cpp $(INCDIR) $(LIBDIR) $(LIB) 

simulator.o: Simulator.cpp Simulator.h
	$(CXX) $(CFLAGS) -o simulator.o -c Simulator.cpp $(INCDIR) $(LIBDIR) $(LIB)


clean: 
	rm -f simulator.o
	rm -f main.o
	rm -r ALL


.PHONY: simulator.o clean
