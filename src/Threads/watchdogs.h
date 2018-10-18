#ifndef _H_WATCHDOG_THREAD_
#define _H_WATCHDOG_THREAD_

#include "ros/ros.h"
#include "../structs.h"
#include "../globals.h"

// Function that checks whether there are new incoming odometry messages
void odomWatchdog(const double &timeout);

// Function that checks whether there are new incoming joystick messages
void joyWatchdog(const double &timeout);

#endif