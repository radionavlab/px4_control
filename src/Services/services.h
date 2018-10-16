
#include "ros/ros.h"
#include "mg_msgs/updatePx4param.h"
#include "std_srvs/Trigger.h"
#include "../PosControl/PosControl.h"
#include "../structs.h"
#include "../globals.h"

//Update values in controller such as PID terms
bool updatePosControlParam(mg_msgs::updatePx4param::Request &req,
	                       mg_msgs::updatePx4param::Response &res);

//Update mass, thrustRatio and gravity
bool updateSystemParam(mg_msgs::updatePx4param::Request &req,
	                   mg_msgs::updatePx4param::Response &res);

//Service request to set the quad into PVA listen mode
bool setQuadPVAMode(std_srvs::Trigger::Request &req,
	                std_srvs::Trigger::Response &res);

//Service request to disarm the quad
bool disarmQuad(std_srvs::Trigger::Request &req,
	            std_srvs::Trigger::Response &res);

//Service request to land the quad
bool landQuad(std_srvs::Trigger::Request &req,
	          std_srvs::Trigger::Response &res);

//Switch position controller to px4 (instead of using locally built controller)
bool switcPosController2PX4(std_srvs::Trigger::Request &req,
	                        std_srvs::Trigger::Response &res);

//Switch position controller to locally-built position control
bool switcPosController2local(std_srvs::Trigger::Request &req,
	                          std_srvs::Trigger::Response &res);