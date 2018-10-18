#include "watchdogs.h"


// Function that checks whether there are new incoming odometry messages
void odomWatchdog(const double &timeout) {
	ROS_INFO("odomWatchdog thread has started!");

	ros::Rate r(10); // 10 hz
	nav_msgs::Odometry localOdom;		//Save odometry data locally
	StateMachine localFSM;				//Save state machine data locally
	watchdogTimeouts localWatchdogs;	//Save watchdog data locally

	while (1)
	{
		//Check if thread should be terminated
		if(WaitForEvent(syncEvents.Terminate,0) == 0){
			break;
		}

		//Get odometry info
	    pthread_mutex_lock(&mutexes.odom);
	    	localOdom = odom;
	    pthread_mutex_unlock(&mutexes.odom);

	    //Get watchdog info
	    pthread_mutex_lock(&mutexes.watchdog);
	    	localWatchdogs = watchdogs;
	    pthread_mutex_unlock(&mutexes.watchdog);

	    //Get information about state of the system
	    pthread_mutex_lock(&mutexes.FSM);
	    	localFSM = FSM;
	    pthread_mutex_unlock(&mutexes.FSM);

	    // ROS_WARN("watchdog: last measurement: %4.2f seconds ago", (ros::Time::now() - localOdom.header.stamp).toSec());
	    if ((ros::Time::now() - localOdom.header.stamp).toSec() > timeout) {
	    	if(localWatchdogs.odom_timeout == false)  {
	    		if((localFSM.State == localFSM.MODE_POSITION_JOY) ||
		   		   (localFSM.State == localFSM.MODE_POSITION_ROS)) {
	    			ROS_WARN("Odometry data is older than %4.1f seconds. Switching to autoland using px4 controller!", timeout);
	    			SetEvent(triggerEvents.use_px4_pos_controller);
	    			SetEvent(triggerEvents.land_quad);
	    		} else if (localFSM.State == localFSM.MODE_AUTOLAND) {
	    			ROS_WARN("Odometry data is older than %4.1f seconds. Switching to px4 controller for finishing to land!", timeout);
	    			SetEvent(triggerEvents.use_px4_pos_controller);
	    		} else {
	    			ROS_WARN("Odometry data is older than %4.1f seconds. Position control is disabled.", timeout);
	    		}
	    	}

	    	// Set timeout to true
		    pthread_mutex_lock(&mutexes.watchdog);
		    	watchdogs.odom_timeout = true;
		    pthread_mutex_unlock(&mutexes.watchdog);
	    } else {
	    	if (localWatchdogs.odom_timeout == true) {
	    		ROS_INFO("Odometry data found! Position control is allowed!");
	    	}
	    	// Set timeout to false
		    pthread_mutex_lock(&mutexes.watchdog);
		    	watchdogs.odom_timeout = false;
		    pthread_mutex_unlock(&mutexes.watchdog);
	    }

		r.sleep();
	}

	//Shutdown here
	pthread_mutex_lock(&mutexes.threadCount);
        threadCount -= 1;
    pthread_mutex_unlock(&mutexes.threadCount);
	// pthread_exit(NULL);
}

// Function that checks whether there are new incoming joystick messages
void joyWatchdog(const double &timeout) {
	ROS_INFO("joyWatchdog thread has started!");

	ros::Rate r(10); // 10 hz
	joyStruct localJoy; 				//Save joystick data locally
	StateMachine localFSM;				//Save state machine data locally
	watchdogTimeouts localWatchdogs;	//Save watchdog data locally

	while (1)
	{
		//Check if thread should be terminated
		if(WaitForEvent(syncEvents.Terminate,0) == 0){
			break;
		}

		//Get latest joystick values
		pthread_mutex_lock(&mutexes.joy);
	    	localJoy = joy;
	    pthread_mutex_unlock(&mutexes.joy);

	    //Get watchdog info
	    pthread_mutex_lock(&mutexes.watchdog);
	    	localWatchdogs = watchdogs;
	    pthread_mutex_unlock(&mutexes.watchdog);

	    //Get information about state of the system
	    pthread_mutex_lock(&mutexes.FSM);
	    	localFSM = FSM;
	    pthread_mutex_unlock(&mutexes.FSM);

	    // ROS_WARN("watchdog: last measurement: %4.2f seconds ago", (ros::Time::now() - localJoy.header.stamp).toSec());
	    if ((ros::Time::now() - localJoy.stamp).toSec() > timeout) {
	    	if(localWatchdogs.joy_timeout == false)  {
	    		if((localFSM.State == localFSM.MODE_POSITION_JOY) ||
		   		   (localFSM.State == localFSM.MODE_POSITION_ROS)) {
	    			ROS_WARN("Joystick data is older than %4.1f seconds. Switching to autoland!", timeout);
	    			SetEvent(triggerEvents.land_quad);
	    		} else if (localFSM.State == localFSM.MODE_AUTOLAND) {
	    			ROS_WARN("Joystick data is older than %4.1f seconds. Completing Autoland and disabling position control...", timeout);
	    		} else {
	    			ROS_WARN("Joystick data is older than %4.1f seconds. Position control is disabled.", timeout);
	    		}
	    	}

	    	// Set timeout to true
		    pthread_mutex_lock(&mutexes.watchdog);
		    	watchdogs.joy_timeout = true;
		    pthread_mutex_unlock(&mutexes.watchdog);
	    } else {
	    	if (localWatchdogs.joy_timeout == true) {
	    		ROS_INFO("Joystick data found!");
	    	}
	    	// Set timeout to false
		    pthread_mutex_lock(&mutexes.watchdog);
		    	watchdogs.joy_timeout = false;
		    pthread_mutex_unlock(&mutexes.watchdog);
	    }

		r.sleep();
	}

	//Shutdown here
	pthread_mutex_lock(&mutexes.threadCount);
        threadCount -= 1;
    pthread_mutex_unlock(&mutexes.threadCount);
	// pthread_exit(NULL);
}
