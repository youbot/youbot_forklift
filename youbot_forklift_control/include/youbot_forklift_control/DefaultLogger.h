/*
 * DefaultLogger.h
 *
 *  Created on: May 10, 2016
 *      Author: Locomotec
 */

#ifndef YOUBOT_FORKLIFT_CONTROL_SRC_DEFAULTLOGGER_H_
#define YOUBOT_FORKLIFT_CONTROL_SRC_DEFAULTLOGGER_H_


#include "youbot_forklift_control/Logger.h"

namespace youbot_forklift_control {

class DefaultLogger: public Logger {
public:
	DefaultLogger();
	virtual ~DefaultLogger();

	void printError(const std::string& errorMsg);
	void printWarning(const std::string& warningMsg);
	void printInfo(const std::string& infoMsg);
};

} /* namespace youbot_forklift_control */

#endif /* YOUBOT_FORKLIFT_CONTROL_SRC_DEFAULTLOGGER_H_ */
