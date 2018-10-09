#ifndef _H_COMMPUB_THREAD_
#define _H_COMMPUB_THREAD_

#include "ros/ros.h"
#include "../JoyDrivers/joyDrivers.h"
#include "../PosControl/PosControlKinetic.h"
#include "../structs.h"
#include "../globals.h"

//Thread for triggering commPubTask
void *commPubTimer(void *threadID);

//Thread for sending reference commands to PX4 (attitude / position)
void *commPubTask(void *threadID);

#endif