#include "FSMTask.h"

void FSMTask(){
	ROS_INFO("State Machine Thread started!");

	StateMachine localFSM;	//Save state machine data locally
	mavros_msgs::State localPX4state;
	watchdogTimeouts localWatchdogs;	//Save watchdog data locally

	ros::NodeHandle n;  
	
	//Create service clients
	ros::ServiceClient armClient = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient SetModeClient = n.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	mavros_msgs::CommandBool ArmMsg;
	ArmMsg.request.value = true;
	mavros_msgs::SetMode ModeMsg;
    ModeMsg.request.custom_mode = "OFFBOARD";

    ros::Time last_request = ros::Time::now();

	while(1){
		WaitForEvent(syncEvents.Timeout,50); //Execute every 100ms

		//Check if thread should be terminated; if so, turn motors off
		if(WaitForEvent(syncEvents.Terminate, 0) == 0){
			ArmMsg.request.value = false;
			armClient.call(ArmMsg);
			break;
		}

		//Get watchdog info
		bool pos_mode_enabled;
	    pthread_mutex_lock(&mutexes.watchdog);
	    	localWatchdogs = watchdogs;
	    pthread_mutex_unlock(&mutexes.watchdog);
	    if ((localWatchdogs.odom_timeout ==  true) || (localWatchdogs.joy_timeout ==  true)) {
	    	pos_mode_enabled = false;
	    } else {
	    	pos_mode_enabled = true;
	    }

		//Check events for change of state and print when changing states
		if(WaitForEvent(joyEvents.buttonA, 0) == 0){
		    pthread_mutex_lock(&mutexes.FSM);
		    	if(FSM.State != FSM.MODE_DISARM){
			    	ROS_INFO("Disarming...");
			    	FSM.State = FSM.MODE_DISARM;
			    }
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(joyEvents.buttonB, 0) == 0){
		    pthread_mutex_lock(&mutexes.FSM);
			    if((FSM.State != FSM.MODE_POSITION_ROS) && pos_mode_enabled){
			    	ROS_INFO("ROS Position Mode!");
			    	FSM.State = FSM.MODE_POSITION_ROS;
			    } else if(localWatchdogs.odom_timeout) {
			    	ROS_INFO("Cannot switch to position mode: odometry data is old!");
			    } else if(localWatchdogs.odom_timeout) {
			    	ROS_INFO("Cannot switch to position mode: joystick data is old!");
			    }
		    pthread_mutex_unlock(&mutexes.FSM);
		    // e_ROS_PosModeSet = 1;   %Flag that tells the RefPubThread that this mode was just enabled
		}
		if(WaitForEvent(joyEvents.buttonX, 0) == 0){
		    pthread_mutex_lock(&mutexes.FSM);
		    	if((FSM.State != FSM.MODE_POSITION_JOY) && pos_mode_enabled) {
			    	ROS_INFO("Joystick Position Mode!");
			    	FSM.State = FSM.MODE_POSITION_JOY;
			    } else if(localWatchdogs.odom_timeout) {
			    	ROS_INFO("Cannot switch to position mode: odometry data is old!");
			    } else if(localWatchdogs.odom_timeout) {
			    	ROS_INFO("Cannot switch to position mode: joystick data is old!");
			    }
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(joyEvents.buttonY, 0) == 0){
		    pthread_mutex_lock(&mutexes.FSM);
		    	if(FSM.State != FSM.MODE_ATTITUDE){
			    	ROS_INFO("Joystick Attitude Mode!");
			    	FSM.State = FSM.MODE_ATTITUDE;
			    }
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(joyEvents.buttonStart, 0) == 0){
		    pthread_mutex_lock(&mutexes.FSM);
		    	if(FSM.State != FSM.MODE_AUTOLAND){
			    	ROS_INFO("Autoland Mode!");
			    	FSM.State = FSM.MODE_AUTOLAND;
			    }
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(joyEvents.buttonLeft, 0) == 0){
			ROS_INFO("SE3 Position Control Mode!");
		    pthread_mutex_lock(&mutexes.FSM);
		    	if(FSM.PosControlMode == FSM.POS_CONTROL_LOCAL){
		    		if(FSM.PosRefMode != FSM.POS_REF_WORLD){
			    		FSM.PosRefMode = FSM.POS_REF_WORLD;
			    		ROS_INFO("Joystick reference is in world frame!");
			    	}
			    	else{
			    		FSM.PosRefMode = FSM.POS_REF_BODY;
			    		ROS_INFO("Joystick reference is in body frame!");
			    	}
			    }
			    FSM.PosControlMode = FSM.POS_CONTROL_LOCAL;
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(joyEvents.buttonRight, 0) == 0){
			ROS_INFO("PX4 Position Control Mode!");
		    pthread_mutex_lock(&mutexes.FSM);
		    	if(FSM.PosControlMode == FSM.POS_CONTROL_PX4){
		    		if(FSM.PosRefMode != FSM.POS_REF_WORLD){
			    		FSM.PosRefMode = FSM.POS_REF_WORLD;
			    		ROS_INFO("Joystick reference is in world frame!");
			    	}
			    	else{
			    		FSM.PosRefMode = FSM.POS_REF_BODY;
			    		ROS_INFO("Joystick reference is in body frame!");
			    	}
			    }
		    	FSM.PosControlMode = FSM.POS_CONTROL_PX4;
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(joyEvents.buttonSelect, 0) == 0){
			ROS_INFO("Terminating Node!");
		    SetEvent(syncEvents.Terminate);
		}

		// Events triggered by other threads
		if(WaitForEvent(triggerEvents.disarm_quad, 0) == 0) {
			ROS_INFO("Disarming...");
		    pthread_mutex_lock(&mutexes.FSM);
		    	FSM.State = FSM.MODE_DISARM;
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(triggerEvents.switch2ros_position_mode, 0) == 0) {
		    pthread_mutex_lock(&mutexes.FSM);
			    if((FSM.State != FSM.MODE_POSITION_ROS) && pos_mode_enabled){
			    	ROS_INFO("ROS Position Mode!");
			    	FSM.State = FSM.MODE_POSITION_ROS;
			    } else if(localWatchdogs.odom_timeout) {
			    	ROS_INFO("Cannot switch to position mode: odometry data is old!");
			    } else if(localWatchdogs.odom_timeout) {
			    	ROS_INFO("Cannot switch to position mode: joystick data is old!");
			    }
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(triggerEvents.switch2joy_position_mode, 0) == 0) {
			ROS_INFO("Joystick Position Mode!");
		    pthread_mutex_lock(&mutexes.FSM);
		    	if((FSM.State != FSM.MODE_POSITION_JOY) && pos_mode_enabled) {
			    	ROS_INFO("Joystick Position Mode!");
			    	FSM.State = FSM.MODE_POSITION_JOY;
			    } else if(localWatchdogs.odom_timeout) {
			    	ROS_INFO("Cannot switch to position mode: odometry data is old!");
			    } else if(localWatchdogs.odom_timeout) {
			    	ROS_INFO("Cannot switch to position mode: joystick data is old!");
			    }
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(triggerEvents.land_quad, 0) == 0) {
		    pthread_mutex_lock(&mutexes.FSM);
		    	if(FSM.State != FSM.MODE_AUTOLAND){
			    	ROS_INFO("Autoland Mode!");
			    	FSM.State = FSM.MODE_AUTOLAND;
			    }
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(triggerEvents.use_px4_pos_controller, 0) == 0) {
			ROS_INFO("Switching position controller to PX4...");
		    pthread_mutex_lock(&mutexes.FSM);
		    	FSM.PosControlMode = FSM.POS_CONTROL_PX4;
		    pthread_mutex_unlock(&mutexes.FSM);
		}
		if(WaitForEvent(triggerEvents.use_local_pos_controller, 0) == 0) {
			ROS_INFO("Switching position controller to SE3 (local)...");
		    pthread_mutex_lock(&mutexes.FSM);
		    	FSM.PosControlMode = FSM.POS_CONTROL_LOCAL;
		    pthread_mutex_unlock(&mutexes.FSM);
		}


		//Get information about state of the system
	    pthread_mutex_lock(&mutexes.FSM);
	    	localFSM = FSM;
	    pthread_mutex_unlock(&mutexes.FSM);

	    //Get PX4 state
	    pthread_mutex_lock(&mutexes.PX4state);
	    	localPX4state = PX4state;
	    pthread_mutex_unlock(&mutexes.PX4state);

	    //Request to arm depending on desired state
	    //Chunk of code extracted from 
	    if((localFSM.State == localFSM.MODE_POSITION_JOY) ||
	       (localFSM.State == localFSM.MODE_POSITION_ROS) ||
	       (localFSM.State == localFSM.MODE_AUTOLAND) ||
	       (localFSM.State == localFSM.MODE_ATTITUDE)){

	        if( localPX4state.mode != "OFFBOARD" &&
	            (ros::Time::now() - last_request > ros::Duration(1.0))){
	            if( SetModeClient.call(ModeMsg) &&
	                ModeMsg.response.success){
	                ROS_INFO("Offboard enabled!");
	            }
	            last_request = ros::Time::now();
	        } else if ((localFSM.State == localFSM.MODE_AUTOLAND) && !localPX4state.armed){
	        	// In autoland, quad disarms automatically when lands. I don't want to rearm
	        	// after disarming in autoland
	        	SetEvent(triggerEvents.disarm_quad);
	        } else if( !localPX4state.armed &&
	                (ros::Time::now() - last_request > ros::Duration(1.0))){
                ArmMsg.request.value = true;
                if( armClient.call(ArmMsg) &&
                    ArmMsg.response.success){
                    ROS_INFO("Vehicle armed!");
                }
                last_request = ros::Time::now();
        	}
        }
	    else {
	    	ArmMsg.request.value = false;
	    	armClient.call(ArmMsg);
	    }

	}

	ROS_INFO("Exiting State Machine Thread...");
	
	pthread_mutex_lock(&mutexes.threadCount);
        threadCount -= 1;
    pthread_mutex_unlock(&mutexes.threadCount);
	// pthread_exit(NULL);
}