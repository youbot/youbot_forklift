/*
 * ForkliftRosHelloWorld.cpp
 *
 *  Created on: May 1, 2016
 *      Author: Locomotec
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <actionlib/client/simple_action_client.h>
#include <youbot_forklift_ros_interface/GoToPositionAction.h>

typedef actionlib::SimpleActionClient<youbot_forklift_ros_interface::GoToPositionAction> Client;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const youbot_forklift_ros_interface::GoToPositionResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]" , state.toString().c_str());
  if (result->position_reached){
	  ROS_INFO("Position reached");
  } else {
	  ROS_INFO("Fail to reached position");
  }
  ros::shutdown();
}

void activeCb()
{
  ROS_INFO("Goal just went active");
}

void feedbackCb(const youbot_forklift_ros_interface::GoToPositionFeedbackConstPtr& feedback)
{
	ROS_INFO("Just [%f] to go!" , feedback->position_error);
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "youbot_forklift_ros_hello_world");
  ros::NodeHandle nh;
  ros::Publisher velocityPublisher = nh.advertise<std_msgs::Float64>("forklift/forklift_controller/max_velocity", 1);
  std_msgs::Float64 velocity;
  Client forkliftClient("goToPosAction", true);
  youbot_forklift_ros_interface::GoToPositionGoal goal;
  ROS_INFO("Waiting for action server to start.");
  forkliftClient.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  velocity.data = 0.006;
  velocityPublisher.publish(velocity);

  goal.goal_position_in_meter = 0.06;
  forkliftClient.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);


  ros::spin();


  return 0;
}
