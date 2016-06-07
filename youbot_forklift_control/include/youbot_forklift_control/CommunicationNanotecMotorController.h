/*
 * CommunicationNanotecMotorController.h
 *
 *  Created on: Dec 9, 2015
 *      Author: Locomotec
 */

#ifndef YOUBOT_FORKLIFT_CONTROL_SRC_COMMUNICATIONWITHNANOTECMOTORCONTROLLER_H_
#define YOUBOT_FORKLIFT_CONTROL_SRC_COMMUNICATIONWITHNANOTECMOTORCONTROLLER_H_

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */

//#include <sstream>
#include "youbot_forklift_control/Logger.h"
#include "youbot_forklift_control/DefaultLogger.h"

#include <memory>

namespace youbot_forklift_control {

class CommunicationNanotecMotorController {
public:

	CommunicationNanotecMotorController(Logger* logger);
	virtual ~CommunicationNanotecMotorController();

	void stopMovementQuick();
	void stopMovementRamp();
	void startMovement();

	void changeAccModeToSinusoidal();
	void changeAccModeToTrapeziodal();
	void changeAccModeToJerkFree();

	void setMaximalSpeedInStepPerS(int speedInStepPerS);
	void setTravelDistanveInSteps(int travelDistanceInSteps);
	void setAccelerationRampInHerzS(int maxAccInHzS);

	void setDirectionToUp();
	void setDirectionToDown();

	void setModeToReferenceRun();
	void setModeToPositionRelative();
	void setModeToPositionAbsolute();
	void setModeToSpeedMode();

	int getMaximalSpeedInStepPerS();
	int getPositionInSteps();

	double getStepSize();

	bool setPhaseCurrent(int percentVelue);
	bool isControllerReferenced();


private:
	int sizeOfEndLineCharacter;
	int maxMessageSize;
	int fileDescriptor;

	bool isInitialized;
	bool isReferenced;

	std::string endOfLineCharacter;
	std::string beginCharacter;

	std::auto_ptr<Logger> logger;

	void setIOForReferenceSensor();
	void setMessageDetails();
	void setRepetitionsToSingleRun();
	void readErrorCode();
	void stopProgram();
	void isMotorReferenced();
	void setStepSize(int stepSize);
	void sendMessage(std::string messageToSend);
	void filterAnswer(std::string& command, std::string& answer);

	void performReferenceProcedure();
	bool initializeConnection();
	bool setAttributesForConnection();

	int stringToInt(std::string stringValue);
	int readControllerStatus();
	std::string addBeginCharacter(std::string stringToChange);
	std::string addEndOfLineCharacter(std::string stringToChange);
	std::string readMessage(std::string& command);
	std::string sendCommandAndReceiveAnswer(std::string messageToAsk);
	std::string sendCommandAndReceiveAnswer(std::string messageToAsk, int commandParameter);
	std::string askForControllerSeries();


};

}

#endif /* YOUBOT_FORKLIFT_CONTROL_SRC_COMMUNICATIONWITHNANOTECMOTORCONTROLLER_H_ */
