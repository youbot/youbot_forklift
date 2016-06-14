/*
 * youbotForkliftControl.h
 *
 *  Created on: Dec 8, 2015
 *      Author: Locomotec
 */

#ifndef YOUBOT_FORKLIFT_CONTROL_YOUBOTFORKLIFTCONTROL_H_
#define YOUBOT_FORKLIFT_CONTROL_YOUBOTFORKLIFTCONTROL_H_

#include "CommunicationNanotecMotorController.h"
#include <memory>
#include <math.h>

namespace youbot_forklift_control {

class YoubotForkliftControl {
public:
	YoubotForkliftControl(Logger* logger = new DefaultLogger);
	virtual ~YoubotForkliftControl();

	double getCurrentAbsolutePosition(); //in meters
	double getActualMaxVelocity();
	bool moveToAbsolutePosition(double absolutePosInMeter);
	bool moveToRelativePosition(double relativePosInMeter);
	bool setMaxVelocity(double speedInMetersperS);
	bool isInitialized();
	void stopMovement();
	void stopMovementRamp();
	void goToUpPosition();
	void goToDownPosition();

	void changeAccModeToSinusoidal();
	void changeAccModeToTrapezoidal();
	void changeAccModeToJerkFree();

	bool setPhaseCurrent(int percentValue);

private:
	int mmPerRev;
	double lengthOfRodInMeters;
	double stepSize;
	double degPerStep;
	double meterPerStep;
	double stepPerMeter;
	double maxSpeedInMetersPerS;
	std::auto_ptr<Logger> logger;
	CommunicationNanotecMotorController motorController;
	void setMotorParameters();
	void setTravelDistance(double travelDistanceInMm);
	double stepsToMeters(int ticks);
	int metersTosteps(double meters);
	bool checkIfRelativeMovementIsPossible(double displacementInMeters);
	bool checkIfAbsoluteMovementIsPossible(double endPosition);
};

}
#endif /* YOUBOT_FORKLIFT_CONTROL_YOUBOTFORKLIFTCONTROL_H_ */
