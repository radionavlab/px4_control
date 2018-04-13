
#include "mg_msgs/updatePx4param.h"
#include "../PosControl/PosControl.h"
#include "ros/ros.h"
#include "../structs.h"
#include "../globals.h"

//Update values in controller such as PID terms
bool updatePosControlParam(mg_msgs::updatePx4param::Request &req,
	                       mg_msgs::updatePx4param::Response &res);

//Update mass, thrustRatio and gravity
bool updateSystemParam(mg_msgs::updatePx4param::Request &req,
	                   mg_msgs::updatePx4param::Response &res);