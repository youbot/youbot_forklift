/*
 * DefaultLogger.cpp
 *
 *  Created on: May 10, 2016
 *      Author: Locomotec
 */

#include "youbot_forklift_control/DefaultLogger.h"

namespace youbot_forklift_control {

DefaultLogger::DefaultLogger() {
	// TODO Auto-generated constructor stub

}

DefaultLogger::~DefaultLogger() {
	// TODO Auto-generated destructor stub
}

void DefaultLogger::printError(const std::string& errorMsg) {
	//perror(errorMsg.c_str());
	std::cout<<"Foklift error: "<<errorMsg<<std::endl;
}

void DefaultLogger::printWarning(const std::string& warningMsg) {
	std::cout<<"Forklift warning: "<<warningMsg<<std::endl;
}

void DefaultLogger::printInfo(const std::string& infoMsg) {
	std::cout<<"Forklift info: "<<infoMsg<<std::endl;
}

} /* namespace youbot_forklift_control */
