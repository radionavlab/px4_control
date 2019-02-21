#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include "std_msgs/Float64.h"
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"
#include "mg_msgs/PVA.h"

#include "structs.h"

// All global variables within the code (mutex-protected)
extern PVA_structure PVA_ref;
extern mavros_msgs::State PX4state;
extern nav_msgs::Odometry odom;
extern joyStruct joy;
extern std::string joyDriver;
extern mg_msgs::PVA PVA_Ros;
extern joyEventList joyEvents;
extern syncEventList syncEvents;
extern triggerEventList triggerEvents;
extern StateMachine FSM;
extern int threadCount;
extern PID_3DOF PosPID;
extern PosControlParam ControlParam;
extern watchdogTimeouts watchdogs;
extern double land_speed;
extern geometry_msgs::PoseStamped RvizPoseRef_;

// Mutexes
extern mutexStruct mutexes;