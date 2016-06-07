/*
 * RosLogger.cpp
 *
 *  Created on: May 10, 2016
 *      Author: Locomotec
 */

#include "youbot_forklift_ros_interface/RosLogger.h"

namespace youbot_forklift_ros_interface {

RosLogger::RosLogger() {
}

RosLogger::~RosLogger() {
}


void RosLogger::printError(const std::string& errorMsg) {
	ROS_ERROR_STREAM(errorMsg);
}

void RosLogger::printWarning(const std::string& warningMsg) {
	ROS_INFO_STREAM(warningMsg);
}

void RosLogger::printInfo(
		const std::string& infoMsg) {
	ROS_INFO_STREAM(infoMsg);
}


}
