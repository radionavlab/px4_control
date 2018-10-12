#include "commPubKinetic.h"
#include <mavros_msgs/AttitudeTarget.h>

void *commPubTimer(void *threadID){
	
	ROS_INFO("commPubTimer has started!");
	int SamplingTime = 4;	//Sampling time in milliseconds

	while(1){
		WaitForEvent(syncEvents.Timeout,SamplingTime);

		//Check if thread should be terminated
		if(WaitForEvent(syncEvents.Terminate,0) == 0){
			break;
		}

		SetEvent(syncEvents.CommPub_trigger);
	}
	
	ROS_INFO("Exiting commPubTimer...");

	//Shutdown here
	pthread_mutex_lock(&mutexes.threadCount);
        threadCount -= 1;
    pthread_mutex_unlock(&mutexes.threadCount);
	pthread_exit(NULL);
}

void *commPubTask(void *threadID){
ROS_INFO("Command Publisher started!");

	//Local variables
	joyStruct localJoy; 			//Used to save global variable into local scope
	StateMachine localFSM;			//Save state machine data locally
	nav_msgs::Odometry localOdom;	//Save odometry data locally
	PVA_structure localPVA_ref;
	PosControlParam localParam;
	geometry_msgs::PoseStamped  PoseRef, RvizPoseRef;
	mavros_msgs::AttitudeTarget AttRef;
	AttRef.type_mask = AttRef.IGNORE_ROLL_RATE +
					   AttRef.IGNORE_PITCH_RATE +
					   AttRef.IGNORE_YAW_RATE;

	//Publishers
	ros::NodeHandle n; 
	ros::Publisher PosPub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",100);
    ros::Publisher AttPub = n.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",100);
    ros::Publisher RvizPosPub = n.advertise<geometry_msgs::PoseStamped>("px4_control_node/local_setpoint",100);

	ros::Time t_prev = ros::Time::now();
	ros::Time t_now = ros::Time::now();
	ros::Duration dt;

	int count = 0;
	while(1){
		WaitForEvent(syncEvents.CommPub_trigger,500);

		//Check if thread should be terminated
		if(WaitForEvent(syncEvents.Terminate,0) == 0){
			break;
		}

		//Ellapsed time calculations
		t_now = ros::Time::now();
		dt = t_now - t_prev;
		t_prev = t_now;
		
		//Get latest joystick values
		pthread_mutex_lock(&mutexes.joy);
	    	localJoy = joy;
	    pthread_mutex_unlock(&mutexes.joy);

	    //Get information about state of the system
	    pthread_mutex_lock(&mutexes.FSM);
	    	localFSM = FSM;
	    pthread_mutex_unlock(&mutexes.FSM);

		//Get odometry info
	    pthread_mutex_lock(&mutexes.odom);
	    	localOdom = odom;
	    pthread_mutex_unlock(&mutexes.odom);

	    //Get reference PVA
	    if(localFSM.State == localFSM.MODE_POSITION_ROS){
		    pthread_mutex_lock(&mutexes.PVA_ros);
		    	localPVA_ref.Pos.pose.position = PVA_Ros.Pos;
		    	localPVA_ref.Pos.pose.orientation = 
							rpy2quat(SetVector3(0, 0, PVA_Ros.yaw));
		    	localPVA_ref.Vel.twist.linear = PVA_Ros.Vel;
		    	localPVA_ref.Acc.accel.linear = PVA_Ros.Acc;
		    pthread_mutex_unlock(&mutexes.PVA_ros);
	    }
	    else{
		    pthread_mutex_lock(&mutexes.PVAref);
		    	localPVA_ref = PVA_ref;
		    pthread_mutex_unlock(&mutexes.PVAref);
	    }


	    //Get position controller parameters
	    pthread_mutex_lock(&mutexes.PID_Param);
	    	localParam = ControlParam;
	    pthread_mutex_unlock(&mutexes.PID_Param);
	    
	    //Use appropriate settings depending on mode
		if((localFSM.State == localFSM.MODE_POSITION_JOY) ||
		   (localFSM.State == localFSM.MODE_POSITION_ROS)){
		   	if (localFSM.PosControlMode == localFSM.POS_CONTROL_LOCAL){
		   		pthread_mutex_lock(&mutexes.PID_Pos);
					PosController(localOdom, localPVA_ref, localParam, dt.toSec(),
		                          PosPID, AttRef);
				pthread_mutex_unlock(&mutexes.PID_Pos);
				PoseRef.pose.position = localPVA_ref.Pos.pose.position;
				RvizPoseRef.pose.position = PoseRef.pose.position;
				RvizPoseRef.pose.orientation = AttRef.orientation;
		   	}
		   	else if (localFSM.PosControlMode == localFSM.POS_CONTROL_PX4){
				PoseRef.pose.orientation = setQuat(0, 0, 0, 1);
				PoseRef.pose = localPVA_ref.Pos.pose;
				RvizPoseRef.pose = PoseRef.pose;
		   	}
		}
		else if(localFSM.State == localFSM.MODE_ATTITUDE){
	    	PoseRef.pose = localPVA_ref.Pos.pose;
			AttRef.orientation = PoseRef.pose.orientation;
			AttRef.thrust = localPVA_ref.thrustRef.data;
			RvizPoseRef.pose = PoseRef.pose;
		}
		else{
	    	PoseRef.pose = localOdom.pose.pose;
			AttRef.orientation = PoseRef.pose.orientation;
			AttRef.thrust = 0.0;
			RvizPoseRef.pose = PoseRef.pose;
		}

		//Set header for position reference
		PoseRef.header.seq = count;
		PoseRef.header.stamp = t_now;
		PoseRef.header.frame_id = "fcu";

		//Set header for attitude reference
		AttRef.header.seq = count;
		AttRef.header.stamp = t_now;
		AttRef.header.frame_id = "fcu";

		// Set header for Rviz publisher
		RvizPoseRef.header.seq = count;
		RvizPoseRef.header.stamp = t_now;
		RvizPoseRef.header.frame_id = "map";

		//Publish references
		if ((localFSM.PosControlMode == localFSM.POS_CONTROL_LOCAL) ||
		    (localFSM.State == localFSM.MODE_ATTITUDE)){
   			AttPub.publish(AttRef);
		}
		else if (localFSM.PosControlMode == localFSM.POS_CONTROL_PX4){
			PosPub.publish(PoseRef);
		}
		RvizPosPub.publish(RvizPoseRef);


   		count += 1;
	}

	ROS_INFO("Exiting Command Publisher Thread...");

	pthread_mutex_lock(&mutexes.threadCount);
        threadCount -= 1;
    pthread_mutex_unlock(&mutexes.threadCount);
	pthread_exit(NULL);

}