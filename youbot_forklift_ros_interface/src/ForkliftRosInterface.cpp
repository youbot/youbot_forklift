#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include "youbot_forklift_control/YoubotForkliftControl.h"
#include <sensor_msgs/JointState.h>
#include <youbot_forklift_ros_interface/GoToPositionAction.h>

#include <actionlib/server/simple_action_server.h>
#include <youbot_forklift_ros_interface/RosLogger.h>

#include <std_msgs/Float32.h>

namespace youbot_forklift_ros_interface {

ros::Subscriber positionSubscriber;
ros::Subscriber velocitySubscriber;
ros::Subscriber stopSubscriber;
ros::Subscriber phaseCurrentSubscriber;
ros::Publisher jointStatePublisher;
sensor_msgs::JointState forkliftJointState;
youbot_forklift_control::YoubotForkliftControl forkliftControl(new RosLogger);
double currentVelocity;
double currentGoalPosition;
double currentPhaseCurrent;

double positionTreshold;

boost::scoped_ptr<actionlib::SimpleActionServer<youbot_forklift_ros_interface::GoToPositionAction> > goToPosAction;
std::string action_name_;
youbot_forklift_ros_interface::GoToPositionFeedback feedback_;
youbot_forklift_ros_interface::GoToPositionResult result_;

/*
// possible callback function if no action shall be used
void positionCallback(const std_msgs::Float64& position){
	if(position.data != currentGoalPosition){
		if (forkliftControl.moveToAbsolutePosition(position.data)){
			currentGoalPosition = position.data;
		}
	}
	else{
		ROS_INFO_STREAM("New goal position is the same as current goal position.");
	}
}
*/

void phaseCurrentCallback(const std_msgs::Int16& phaseCurrent){
	if(phaseCurrent.data != currentPhaseCurrent){
		if (forkliftControl.setPhaseCurrent(phaseCurrent.data)){
			ROS_INFO_STREAM("Phase current was changed.");
			currentPhaseCurrent=phaseCurrent.data;
		}
		else{
			ROS_INFO_STREAM("Phase current can not be changed.");
		}
	}
	else{
		ROS_INFO_STREAM("New phase current value is the same as actual one.");
	}
}

void velocityCallback(const std_msgs::Float64& velocity){
	if(velocity.data != currentVelocity){
		if(forkliftControl.setMaxVelocity(fabs(velocity.data))){
			currentVelocity = velocity.data;
			ROS_INFO_STREAM("Max velocity set to: " << velocity.data);
		}
	}
	else{
		ROS_INFO_STREAM("Max velocity set to: " << velocity.data);
	}
}

bool goToPosition(double position){
	if(position != currentGoalPosition){
		if (forkliftControl.moveToAbsolutePosition(position)){
			currentGoalPosition = position;
			return true;
		}
		else{
			return false;
		}
	}
	else{
		ROS_INFO_STREAM("New goal position is the same as current goal position.");
		return false;
	}
}

void setGoal(){
	result_.position_reached = false;
	double newGoal = goToPosAction->acceptNewGoal()->goal_position_in_meter;
	ROS_INFO_STREAM("New goal for forklift: " << newGoal);
	double position = newGoal-forkliftControl.getCurrentAbsolutePosition();
	if (fabs(newGoal-forkliftControl.getCurrentAbsolutePosition()) < positionTreshold){
		double position = newGoal-forkliftControl.getCurrentAbsolutePosition();
		result_.position_reached = true;
		goToPosAction->setSucceeded(result_);
	} else if (!goToPosition(newGoal)){
		goToPosAction->setAborted(result_);
	}
}

void stopMovement(){
	ROS_INFO_STREAM("The movement was stopped by the user.");
	forkliftControl.stopMovement();
	currentGoalPosition = forkliftControl.getCurrentAbsolutePosition();
	 if (goToPosAction->isActive()){
		 goToPosAction->setPreempted();
	 }
}

void stopMovementCallback(const std_msgs::Empty& empty_msg){
	stopMovement();
}

}

using namespace youbot_forklift_ros_interface;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_plan_single_target");
	ros::NodeHandle nh;
	ros::Rate rate(5); //frequency of loop in Hz
	currentVelocity = 0;
	currentPhaseCurrent = 0;
	currentGoalPosition = forkliftControl.getCurrentAbsolutePosition();

	positionTreshold = 0.001;

	goToPosAction.reset(new actionlib::SimpleActionServer<youbot_forklift_ros_interface::GoToPositionAction> (nh, "goToPosAction", false));
	goToPosAction->registerGoalCallback(setGoal);
	goToPosAction->registerPreemptCallback(stopMovement);

	while (!forkliftControl.isInitialized()){
		ROS_INFO_STREAM("Waiting for controller initialization");
		ros::Duration(0.5).sleep();
	}
	ROS_INFO_STREAM("Controller initialized");
	goToPosAction->start();

	//positionSubscriber = nh.subscribe("forklift/forklift_controller/position_command", 1000, positionCallback);
	velocitySubscriber = nh.subscribe("forklift/forklift_controller/max_velocity", 1000, velocityCallback);
	stopSubscriber = nh.subscribe("forklift/forklift_controller/stopMovement", 1000, stopMovementCallback);
	phaseCurrentSubscriber = nh.subscribe("forklift/forklift_controller/phase_current", 1000, phaseCurrentCallback);
	jointStatePublisher = nh.advertise<sensor_msgs::JointState> ("forklift/joint_states", 1);

	forkliftJointState.name.resize(1);
	forkliftJointState.position.resize(1);
	forkliftJointState.velocity.resize(1);
	forkliftJointState.name[0]= "forklift_axis";
	while (nh.ok()) {
		ros::spinOnce();
	    if (goToPosAction->isActive()){
	    	feedback_.position_error = currentGoalPosition-forkliftControl.getCurrentAbsolutePosition();
	    	goToPosAction->publishFeedback(feedback_);
	    	if (fabs(currentGoalPosition-forkliftControl.getCurrentAbsolutePosition()) < positionTreshold) {
	    		result_.position_reached = true;
	    		goToPosAction->setSucceeded(result_);
	    	}
	    }

		forkliftJointState.header.stamp =  ros::Time::now();
		forkliftJointState.position[0] = forkliftControl.getCurrentAbsolutePosition();
		jointStatePublisher.publish(forkliftJointState);
		rate.sleep();
	}

	ros::shutdown();

	return 0;
}
