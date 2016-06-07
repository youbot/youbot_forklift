/*
 * RosLogger.h
 *
 *  Created on: May 10, 2016
 *      Author: Locomotec
 */

#ifndef YOUBOT_FORKLIFT_ROS_INTERFACE_SRC_ROSLOGGER_H_
#define YOUBOT_FORKLIFT_ROS_INTERFACE_SRC_ROSLOGGER_H_

#include "youbot_forklift_control/Logger.h"
#include <ros/console.h>

namespace youbot_forklift_ros_interface {

class RosLogger: public youbot_forklift_control::Logger {
public:
	RosLogger();
	virtual ~RosLogger();

	void printError(const std::string& errorMsg);
	void printWarning(const std::string& warningMsg);
	void printInfo(const std::string& infoMsg);
};

}

#endif /* YOUBOT_FORKLIFT_ROS_INTERFACE_SRC_ROSLOGGER_H_ */
