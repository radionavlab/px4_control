#include "structs.h"

void initializePVA(PVA_structure &PVA){
	PVA.Pos.pose.orientation = zeroQuaternion();
}

void initializeJoy(joyStruct &Joy){
	Joy.stamp = ros::Time(0.0);
	Joy.buttonA = 0;
	Joy.buttonB = 0;
	Joy.buttonX = 0;
	Joy.buttonY = 0;
	Joy.buttonR1 = 0;
	Joy.buttonL1 = 0;
	Joy.buttonSelect = 0;
	Joy.buttonStart = 0;
	Joy.buttonLeft = 0;
	Joy.buttonRight = 0;
	Joy.buttonUp = 0;
	Joy.buttonDown = 0;
	
	Joy.LstickHor = 0;
	Joy.LstickVer = 0;
	Joy.RstickHor = 0;
	Joy.RstickVer = 0;
	Joy.L2 = 0;
	Joy.R2 = 0;
}

void initializeEvents(joyEventList &JoyEvents,
	                  syncEventList &SyncEvents,
	                  triggerEventList &triggerEvents){
	//Auto-reset events
	JoyEvents.buttonA = neosmart::CreateEvent(false,false);
	JoyEvents.buttonB = neosmart::CreateEvent(false,false);
	JoyEvents.buttonX = neosmart::CreateEvent(false,false);
	JoyEvents.buttonY = neosmart::CreateEvent(false,false);
	JoyEvents.buttonR1 = neosmart::CreateEvent(false,false);
	JoyEvents.buttonL1 = neosmart::CreateEvent(false,false);
	JoyEvents.buttonSelect = neosmart::CreateEvent(false,false);
	JoyEvents.buttonStart = neosmart::CreateEvent(false,false);
	JoyEvents.buttonLeft = neosmart::CreateEvent(false,false);
	JoyEvents.buttonRight = neosmart::CreateEvent(false,false);
	JoyEvents.buttonUp = neosmart::CreateEvent(false,false);
	JoyEvents.buttonDown = neosmart::CreateEvent(false,false);

	SyncEvents.Timeout = neosmart::CreateEvent(false,false);
	SyncEvents.Joy_trigger = neosmart::CreateEvent(false,false);
	SyncEvents.CommPub_trigger = neosmart::CreateEvent(false,false);

	triggerEvents.switch2ros_position_mode = neosmart::CreateEvent(false,false);
	triggerEvents.switch2joy_position_mode = neosmart::CreateEvent(false,false);
	triggerEvents.disarm_quad = neosmart::CreateEvent(false,false);
	triggerEvents.land_quad = neosmart::CreateEvent(false,false);
	triggerEvents.use_px4_pos_controller = neosmart::CreateEvent(false,false);
	triggerEvents.use_local_pos_controller = neosmart::CreateEvent(false,false);

	//Manual reset events
	SyncEvents.Terminate = neosmart::CreateEvent(true,false);
}

void destroyEvents(joyEventList &JoyEvents,
	               syncEventList &SyncEvents,
	               triggerEventList &triggerEvents){
	neosmart::DestroyEvent(JoyEvents.buttonA);
	neosmart::DestroyEvent(JoyEvents.buttonB);
	neosmart::DestroyEvent(JoyEvents.buttonX);
	neosmart::DestroyEvent(JoyEvents.buttonY);
	neosmart::DestroyEvent(JoyEvents.buttonR1);
	neosmart::DestroyEvent(JoyEvents.buttonL1);
	neosmart::DestroyEvent(JoyEvents.buttonSelect);
	neosmart::DestroyEvent(JoyEvents.buttonStart);
	neosmart::DestroyEvent(JoyEvents.buttonLeft);
	neosmart::DestroyEvent(JoyEvents.buttonRight);
	neosmart::DestroyEvent(JoyEvents.buttonUp);
	neosmart::DestroyEvent(JoyEvents.buttonDown);

	neosmart::DestroyEvent(SyncEvents.Timeout);
	neosmart::DestroyEvent(SyncEvents.Terminate);
	neosmart::DestroyEvent(SyncEvents.Joy_trigger);
	neosmart::DestroyEvent(SyncEvents.CommPub_trigger);

	neosmart::DestroyEvent(triggerEvents.switch2ros_position_mode);
	neosmart::DestroyEvent(triggerEvents.switch2joy_position_mode);
	neosmart::DestroyEvent(triggerEvents.disarm_quad);
	neosmart::DestroyEvent(triggerEvents.land_quad);
	neosmart::DestroyEvent(triggerEvents.use_px4_pos_controller);
	neosmart::DestroyEvent(triggerEvents.use_local_pos_controller);
}


void initializeMutexes(mutexStruct &mutexes){
	pthread_mutex_init(&mutexes.PVAref, NULL);
	pthread_mutex_init(&mutexes.PVA_ros, NULL);
	pthread_mutex_init(&mutexes.PX4state, NULL);
	pthread_mutex_init(&mutexes.odom, NULL);
	pthread_mutex_init(&mutexes.joy, NULL);
	pthread_mutex_init(&mutexes.joyEvents, NULL);
	pthread_mutex_init(&mutexes.FSM, NULL);
	pthread_mutex_init(&mutexes.threadCount, NULL);
	pthread_mutex_init(&mutexes.PID_Pos, NULL);
	pthread_mutex_init(&mutexes.PID_Param, NULL);
	pthread_mutex_init(&mutexes.watchdog, NULL);
	pthread_mutex_init(&mutexes.rviz_pose_ref, NULL);
}

void destroyMutexes(mutexStruct &mutexes){
	pthread_mutex_destroy(&mutexes.PVAref);
	pthread_mutex_destroy(&mutexes.PVA_ros);
	pthread_mutex_destroy(&mutexes.PX4state);
	pthread_mutex_destroy(&mutexes.odom);
	pthread_mutex_destroy(&mutexes.joy);
	pthread_mutex_destroy(&mutexes.joyEvents);
	pthread_mutex_destroy(&mutexes.FSM);
	pthread_mutex_destroy(&mutexes.threadCount);
	pthread_mutex_destroy(&mutexes.PID_Pos);
	pthread_mutex_destroy(&mutexes.PID_Param);
	pthread_mutex_destroy(&mutexes.watchdog);
	pthread_mutex_destroy(&mutexes.rviz_pose_ref);
}

void initializeStateMachine(StateMachine &FSM){
	FSM.MODE_DISARM = 0;		//Disarm motors
	FSM.MODE_ATTITUDE = 1;		//Attitude mode
	FSM.MODE_POSITION_JOY = 2;	//Position control with references from joystick
	FSM.MODE_POSITION_ROS = 3;	//Position control with references from a ROS topic
	FSM.MODE_AUTOLAND = 4;		//Velocity control for landing quad

	FSM.POS_CONTROL_LOCAL = 0;	//Control is made through the current node
	FSM.POS_CONTROL_PX4 = 1;	//Control is made through PX4 software

	FSM.POS_REF_WORLD = 0;		//Joysticks references integrate in world frame
	FSM.POS_REF_BODY = 1;		//Joystick references integrate in body frame

	//Initial states
	FSM.State = FSM.MODE_DISARM;
	FSM.PosControlMode = FSM.POS_CONTROL_LOCAL;
	FSM.PosRefMode = FSM.POS_REF_BODY;
}

void printCurrentState(StateMachine FSM){
	if(FSM.State == FSM.MODE_DISARM){
		ROS_INFO("Current FSM state: DISARM");
	}
	else if(FSM.State == FSM.MODE_ATTITUDE){
		ROS_INFO("Current FSM state: ATTITUDE");
	}
	else if(FSM.State == FSM.MODE_POSITION_JOY){
		ROS_INFO("Current FSM state: POSITION JOY");
	}
	else if(FSM.State == FSM.MODE_POSITION_ROS){
		ROS_INFO("Current FSM state: POSITION ROS");
	}
	else if(FSM.State == FSM.MODE_AUTOLAND){
		ROS_INFO("Current FSM state: AUTOLAND");
	}


	if(FSM.PosControlMode == FSM.POS_CONTROL_LOCAL){
		ROS_INFO("Position Control Mode: LOCAL");
	}
	else if(FSM.PosControlMode == FSM.POS_CONTROL_PX4){
		ROS_INFO("Position Control Mode: PX4");
	}
}

