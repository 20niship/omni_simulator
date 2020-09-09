#ifndef SIMULATOR_HEADER_H
#define SIMULATOR_HEADER_H

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <iostream>
#include <array>
#include <algorithm>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawBox     dsDrawBoxD
#define dsDrawLine    dsDrawLineD
#define dsDrawSphere  dsDrawSphereD
#endif

#define THREE_WHEEL
#define SHOW_SIMULATOR_LOG

namespace Simulator{
void init(int argc, char **argv);
void loop();
void printLog();
void setMotorParams(int id, int Kp, int Ki, int Ti, int max_current);
void setMotorCurrent(int id, float current);
void freeMotor(int id);
std::uint16_t getEncTotal(int id);
std::uint16_t getLineSensorColors(int id);


// static void start();
// static void nearCallback(void *, dGeomID, dGeomID);
// static void command(int);
// void control();
// static void simLoop(int);
// void setDrawStuff();
// void makeOmni();

void quit();
};

void control(int);

#endif

