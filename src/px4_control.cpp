#include "ros/ros.h"

#include "structs.h"
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"
#include "Callbacks/callbacks.h"
#include "Threads/FSMTask.h"
#include "Threads/joyTask.h"
#include "Threads/commPub.h"
#include "Services/services.h"
#include "Actions/actions.h"
#include <mg_msgs/follow_PVAJS_trajectoryAction.h>

#include <thread>


// Global variables
PVA_structure PVA_ref;        //Joystick references
mg_msgs::PVA PVA_Ros;     //References from topic
mavros_msgs::State PX4state;
nav_msgs::Odometry odom;
joyStruct joy;
joyEventList joyEvents;
syncEventList syncEvents;
triggerEventList triggerEvents;
mutexStruct mutexes;
StateMachine FSM;
int threadCount = 0;
PID_3DOF PosPID;
PosControlParam ControlParam;
std::string odomTopic, joyDriver, pvaTopic;

int main(int argc, char **argv)
{
  //Initialize ROS
  ros::init(argc, argv, "~");
  ros::NodeHandle n;  

  //Initialize some variables
  initializePVA(PVA_ref);
  initializeJoy(joy);
  initializeEvents(joyEvents, syncEvents, triggerEvents);
  initializeMutexes(mutexes);
  initializeStateMachine(FSM);
  initializePID3(PosPID);
  readROSparameterServer(PosPID, ControlParam);

  //Print initial state of the finite state machine
  printCurrentState(FSM);

  //Get odometry topic and joystick driver
  ros::param::get("px4_control_node/odomTopic", odomTopic);
  ros::param::get("px4_control_node/joyDriver", joyDriver);
  ros::param::get("px4_control_node/pvaTopic", pvaTopic);

  // Autoland speed
  double land_speed;
  ros::param::get("px4_control_node/land_speed", land_speed);

  //Create services ------------------------------------------
  ros::ServiceServer PID_srv = n.advertiseService("px4_control_node/updatePosControlParam", updatePosControlParam);
  ros::ServiceServer Param_srv = n.advertiseService("px4_control_node/updateQuadParam", updateSystemParam);
  ros::ServiceServer PVA_mode_srv = n.advertiseService("px4_control_node/setQuadPVAMode", setQuadPVAMode);
  ros::ServiceServer disarm_srv = n.advertiseService("px4_control_node/disarmQuad", disarmQuad);
  ros::ServiceServer land_srv = n.advertiseService("px4_control_node/landQuad", landQuad);
  ros::ServiceServer use_px4_pos_control_srv = n.advertiseService("px4_control_node/use_px4_pos_controller", switcPosController2PX4);
  ros::ServiceServer use_local_pos_control_srv = n.advertiseService("px4_control_node/use_SE3_pos_controller", switcPosController2local);


  //Create actions -------------------------------------------
  follow_PVAJS_trajectoryAction follow_PVAJS_trajectory_action("follow_PVAJS_trajectory_action");

  //Subscribers ----------------------------------------------
  ros::Subscriber stateSub = n.subscribe("mavros/state", 10, stateCallback);
  ros::Subscriber odomSub = n.subscribe(odomTopic, 10, odomCallback);
  ros::Subscriber joySub = n.subscribe("joy", 10, joyCallback);
  ros::Subscriber PvaSub = n.subscribe(pvaTopic, 10, PVACallback);
  // ros::Subscriber tfSub = n.subscribe(odomTopic, 10, tfCallback);

  //Echo subscriber topics
  ROS_INFO("odomSub topic: %s", odomSub.getTopic().c_str());
  ROS_INFO("PvaSub topic: %s", PvaSub.getTopic().c_str());
  ROS_INFO("joySub topic: %s", joySub.getTopic().c_str());

  // I'll leave this portion commented until I can certify that the quad can compile C++11
  // //Threads --------------------------------------------------
  // pthread_t h_FSMThread;      //Finite state machine
  // pthread_t h_joyThreadTimer; //Timer for joystick thread
  // pthread_t h_joyThread;      //Joystick thread
  // pthread_t h_commPubTimer;   //Timer for command publisher
  // pthread_t h_commPubThread;  //Command publisher thread
  // int ReturnCode;

  // //Start  finite state machine
  // if (ReturnCode = pthread_create(&h_FSMThread, NULL, FSMTask, NULL)){
  //   printf("Start State Machine failed; return code from pthread_create() is %d\n", ReturnCode);
  //   exit(-1);
  // }
  // else{
  //   pthread_mutex_lock(&mutexes.threadCount);
  //       threadCount += 1;
  //   pthread_mutex_unlock(&mutexes.threadCount);
  // }

  // //Start joystick timer thread
  // if (ReturnCode = pthread_create(&h_joyThreadTimer, NULL, joyTaskTimer, NULL)){
  //   printf("Start joystick timer thread failed; return code from pthread_create() is %d\n", ReturnCode);
  //   exit(-1);
  // }
  // else{
  //   pthread_mutex_lock(&mutexes.threadCount);
  //       threadCount += 1;
  //   pthread_mutex_unlock(&mutexes.threadCount);
  // }


  // //Start joystick thread
  // if (ReturnCode = pthread_create(&h_joyThread, NULL, joyTask, NULL)){
  //   printf("Start joystick thread failed; return code from pthread_create() is %d\n", ReturnCode);
  //   exit(-1);
  // }
  // else{
  //   pthread_mutex_lock(&mutexes.threadCount);
  //       threadCount += 1;
  //   pthread_mutex_unlock(&mutexes.threadCount);
  // }

  //   //Start command publisher timer thread
  // if (ReturnCode = pthread_create(&h_commPubTimer, NULL, commPubTimer, NULL)){
  //   printf("Start command publisher timer thread failed; return code from pthread_create() is %d\n", ReturnCode);
  //   exit(-1);
  // }
  // else{
  //   pthread_mutex_lock(&mutexes.threadCount);
  //       threadCount += 1;
  //   pthread_mutex_unlock(&mutexes.threadCount);
  // }


  // //Start command publisher thread
  // if (ReturnCode = pthread_create(&h_commPubThread, NULL, commPubTask, NULL)){
  //   printf("Start command publisher thread failed; return code from pthread_create() is %d\n", ReturnCode);
  //   exit(-1);
  // }
  // else{
  //   pthread_mutex_lock(&mutexes.threadCount);
  //       threadCount += 1;
  //   pthread_mutex_unlock(&mutexes.threadCount);
  // }

  //Threads --------------------------------------------------
  std::thread h_FSMThread;      //Finite state machine
  std::thread h_joyThreadTimer; //Timer for joystick thread
  std::thread h_joyThread;      //Joystick thread
  std::thread h_commPubTimer;   //Timer for command publisher
  std::thread h_commPubThread;  //Command publisher thread

  //Start  finite state machine
  h_FSMThread = std::thread(FSMTask);
  threadCount += 1;

  //Start joystick timer thread
  h_joyThreadTimer = std::thread(joyTaskTimer);
  threadCount += 1;

  //Start joystick thread
  h_joyThread = std::thread(joyTask, land_speed);
  threadCount += 1;

  //Start command publisher timer thread
  h_commPubTimer = std::thread(commPubTimer);
  threadCount += 1;

  //Start command publisher thread
  h_commPubThread = std::thread(commPubTask);
  threadCount += 1;

  //Start loop ----------------------------------------------------
  ros::Rate loop_rate(500);

  int localThreadCount;
  while (ros::ok())
  {

    //Check if all threads were terminated
    pthread_mutex_lock(&mutexes.threadCount);
        localThreadCount = threadCount;
    pthread_mutex_unlock(&mutexes.threadCount);
    if(localThreadCount == 0){
      break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  //Terminate program ---------------------------------------------
  destroyEvents(joyEvents, syncEvents, triggerEvents);
  destroyMutexes(mutexes);

  ROS_INFO("Process Ended with Success!");


  return 0;

}