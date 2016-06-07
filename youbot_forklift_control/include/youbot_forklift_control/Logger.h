/*
 * Logger.h
 *
 *  Created on: May 10, 2016
 *      Author: Locomotec
 */

#ifndef YOUBOT_FORKLIFT_CONTROL_LOGGER_H_
#define YOUBOT_FORKLIFT_CONTROL_LOGGER_H_
#include <string.h>
#include <sstream>
#include <iostream>
#include <stdio.h>

namespace youbot_forklift_control {

class Logger {
public:
	Logger();
	virtual ~Logger();
	virtual void printError(const std::string& errorMsg) = 0;
	virtual void printWarning(const std::string& warningMsg) = 0;
	virtual void printInfo(const std::string& infoMsg) = 0;

};

} /* namespace youbot_forklift_control */

#endif /* YOUBOT_FORKLIFT_CONTROL_LOGGER_H_ */
