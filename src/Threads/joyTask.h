#ifndef _H_JOY_THREAD_
#define _H_JOY_THREAD_

#include "ros/ros.h"
#include "../JoyDrivers/joyDrivers.h"
#include "../structs.h"
#include "../globals.h"

//Function to load parameters that are used to generate joystick references
void loadJoyRefParam(double &RollMax, double &PitchMax, double &YawRateMax,
	                 double &maxThrust, double &xRate, double &yRate, 
	                 double &zRate, double &PosRefTimeConstant);

//Thread for triggering joy task
void joyTaskTimer();

//Thread for handling joystick data
void joyTask(const double land_speed);

#endif