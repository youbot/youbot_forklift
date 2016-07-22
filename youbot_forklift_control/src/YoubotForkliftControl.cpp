/*
 * youbotForkliftControl.cpp
 *
 *  Created on: Dec 8, 2015
 *      Author: Locomotec
 */

#include "youbot_forklift_control/YoubotForkliftControl.h"

namespace youbot_forklift_control {

YoubotForkliftControl::YoubotForkliftControl(Logger* logger_) :motorController(logger_), logger(logger_) {
	maxSpeedInMetersPerS = 0.01;
	setMotorParameters();
}


YoubotForkliftControl::~YoubotForkliftControl() {
}

double YoubotForkliftControl::getCurrentAbsolutePosition() {
	int steps = motorController.getPositionInSteps();
	return stepsToMeters(steps);
}

bool YoubotForkliftControl::moveToAbsolutePosition(double absolutePosInMeter) {
	if (checkIfAbsoluteMovementIsPossible(absolutePosInMeter)){
		motorController.setModeToPositionAbsolute();
		setTravelDistance(absolutePosInMeter);
		motorController.startMovement();
		return true;
	}	else {
		return false;
	}
}

bool YoubotForkliftControl::moveToRelativePosition(double relativePosInMeter) {
	if(checkIfRelativeMovementIsPossible(relativePosInMeter)){
		motorController.setModeToPositionRelative();
		if(relativePosInMeter<=0){
			motorController.setDirectionToUp();
		} else {
			motorController.setDirectionToDown();
		}
		setTravelDistance(fabs(relativePosInMeter));
		motorController.startMovement();
		return true;
	}	else {
		return false;
	}
}

void YoubotForkliftControl::stopMovement() {
	motorController.stopMovementQuick();
}

void YoubotForkliftControl::stopMovementRamp() {
	motorController.stopMovementRamp();
}

void YoubotForkliftControl::setMotorParameters() {
	mmPerRev = 2;
	degPerStep = 1.8;
	stepSize = motorController.getStepSize();
	stepPerMeter = 360 / degPerStep /stepSize / mmPerRev * 1000;
	meterPerStep = 1/stepPerMeter;
	lengthOfRodInMeters = 0.25;
	setMaxVelocity(0.004);
}

bool YoubotForkliftControl::setMaxVelocity(double speedInMetersPerS) {
	if (speedInMetersPerS < 0.003){
		logger->printWarning("Speed must be higher than 0.003 m/s. Speed value will not be changed");
		return false;
	}	else if (speedInMetersPerS > maxSpeedInMetersPerS) {
		logger->printWarning("Given speed is higher than 0.01 m/s. Speed value will not be changed");
		return false;
	}	else {
		motorController.setMaximalSpeedInStepPerS(metersTosteps(speedInMetersPerS));
		return true;
	}
}

bool YoubotForkliftControl::checkIfRelativeMovementIsPossible(double displacementInMeters) {
	double positionAfterMotion = displacementInMeters + getCurrentAbsolutePosition();
	return checkIfAbsoluteMovementIsPossible(positionAfterMotion);
}

void YoubotForkliftControl::goToUpPosition() {
	moveToAbsolutePosition(lengthOfRodInMeters);
}

void YoubotForkliftControl::goToDownPosition() {
	moveToAbsolutePosition(0);
}

double YoubotForkliftControl::getActualMaxVelocity() {
	return stepsToMeters(motorController.getMaximalSpeedInStepPerS());
}

bool YoubotForkliftControl::isInitialized() {
	return motorController.isControllerReferenced();
}

double YoubotForkliftControl::stepsToMeters(int ticks) {
	double meters = ticks * meterPerStep;
	return meters;
}

int YoubotForkliftControl::metersTosteps(double meters) {
	int steps = meters * stepPerMeter;
	return steps;
}

bool YoubotForkliftControl::checkIfAbsoluteMovementIsPossible(double endPosition) {
	if (endPosition < 0){
		logger->printWarning("Position out of range (below 0)");
		return false;
	}	else if (endPosition > lengthOfRodInMeters) {
		std::ostringstream strs;
		strs << "Position out of range (above " << lengthOfRodInMeters << ")";
		logger->printWarning(strs.str());
		return false;
	}	else {
		return true;
	}
}

void  YoubotForkliftControl::changeAccModeToSinusoidal(){
	motorController.changeAccModeToSinusoidal();
}
void  YoubotForkliftControl::changeAccModeToTrapezoidal(){
	motorController.changeAccModeToTrapeziodal();
}
void  YoubotForkliftControl::changeAccModeToJerkFree(){
	motorController.changeAccModeToJerkFree();
}
bool  YoubotForkliftControl::setPhaseCurrent(int percentValue){
	return motorController.setPhaseCurrent(percentValue);
}
void  YoubotForkliftControl::setTravelDistance(double travelDistanceInMeters){
	motorController.setTravelDistanveInSteps(metersTosteps(travelDistanceInMeters));
}

}
