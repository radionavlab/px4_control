#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mg_msgs/follow_PVAJS_trajectoryAction.h>
#include "../globals.h"
#include "../HelperFunctions/helper.h"


class follow_PVAJS_trajectoryAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mg_msgs::follow_PVAJS_trajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  mg_msgs::follow_PVAJS_trajectoryFeedback feedback_;
  mg_msgs::follow_PVAJS_trajectoryResult result_;

public:

  follow_PVAJS_trajectoryAction(std::string name) :
    as_(nh_, name, boost::bind(&follow_PVAJS_trajectoryAction::executeCB, this, _1), false),
    action_name_(name) {
    as_.start();
  }

  ~follow_PVAJS_trajectoryAction(void) { }

  void executeCB(const mg_msgs::follow_PVAJS_trajectoryGoalConstPtr &goal) {
    // Boolean for success
    bool success = true;

    // Get sampling time
    double dt = goal->samplingTime;
    if (dt  <=0) {
      ROS_ERROR("[%s] Cannot execute PVAJS trajectory with sampling time %f", action_name_.c_str(), dt);
      success = false;
      return;
    } else {
      ROS_INFO("Executing trajectory with sampling time %f.", dt);
    }
    // helper variables
    ros::Rate r(1.0/dt);

    //Get information about state of the system
    StateMachine localFSM;  //Save state machine data locally
    pthread_mutex_lock(&mutexes.FSM);
      localFSM = FSM;
    pthread_mutex_unlock(&mutexes.FSM);

    // Cannot switch when in autoland or disarmed!
    if((localFSM.State == localFSM.MODE_DISARM) || (localFSM.State == localFSM.MODE_AUTOLAND)) {
      ROS_WARN("[px4_control_node] Cannot trigger Action: Quad is Disarmed or in Autoland!");
      return;
    }

    // Set quad to listen to PVA references
    SetEvent(triggerEvents.switch2ros_position_mode);

    // Wait until PVA mode is triggered (check every 0.01 seconds)
    ros::Time t0 = ros::Time::now();
    do {
      if ((ros::Time::now() - t0).toSec() > 5.0) {
          ROS_ERROR("[%s] px4_control_node is taking too long to switch to PVA mode! Aborting...",
                     action_name_.c_str());
          return;
      }

      r.sleep();
      pthread_mutex_lock(&mutexes.FSM);
        localFSM = FSM;
      pthread_mutex_unlock(&mutexes.FSM);
    } while (localFSM.State != localFSM.MODE_POSITION_ROS);

    // publish info to the console for the user
    mg_msgs::PVAJS_array flatStates = goal->flatStates;
    ROS_INFO("[%s]: Executing action!", action_name_.c_str());

    // start executing the action
    for(int i=0; i < flatStates.PVAJS_array.size(); i++) {
      // check that halt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();

        // Set quad to stop by switching to joy position mode
        // SetEvent(triggerEvents.switch2joy_position_mode);

        // Fill in the PVA structure
        pthread_mutex_lock(&mutexes.PVA_ros);
          PVA_Ros.Pos = flatStates.PVAJS_array[i].Pos;
          PVA_Ros.Vel = ZeroVector3();
          PVA_Ros.Acc = ZeroVector3();
          PVA_Ros.yaw = 0.0;
        pthread_mutex_unlock(&mutexes.PVA_ros); 

        success = false;
        break;
      }

      // Fill in the PVA structure
      pthread_mutex_lock(&mutexes.PVA_ros);
        PVA_Ros.Pos = flatStates.PVAJS_array[i].Pos;
        PVA_Ros.Vel = flatStates.PVAJS_array[i].Vel;
        PVA_Ros.Acc = flatStates.PVAJS_array[i].Acc;
        PVA_Ros.yaw = flatStates.PVAJS_array[i].yaw;
      pthread_mutex_unlock(&mutexes.PVA_ros); 

      // ROS_INFO("%d/%d", i, int(flatStates.PVAJS_array.size() - 1));

      // publish the feedback
      feedback_.currentFlatState = flatStates.PVAJS_array[i];
      as_.publishFeedback(feedback_);

      // Sleep until publishing the next reference
      r.sleep();
    }

    if(success) {
      ROS_INFO("[%s]: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

}; // namespace mg_msgs
