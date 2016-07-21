/*
 * CommunicationNanotecMotorController.cpp
 *
 *  Created on: Dec 9, 2015
 *      Author: Locomotec
 */

#include"youbot_forklift_control/CommunicationNanotecMotorController.h"

namespace youbot_forklift_control {


CommunicationNanotecMotorController::CommunicationNanotecMotorController(
		Logger* logger) {
	this->logger.reset(logger);
	isReferenced = false;
	isInitialized = false;
	setMessageDetails();
	if (initializeConnection()) {
		isInitialized = true;
		performReferenceProcedure();
	}
}


CommunicationNanotecMotorController::~CommunicationNanotecMotorController() {
	stopMovementQuick();
	close(fileDescriptor);
}

void CommunicationNanotecMotorController::setMessageDetails() {
	endOfLineCharacter ="\r";
	beginCharacter ="#1";
	maxMessageSize = 32;
	sizeOfEndLineCharacter = endOfLineCharacter.size();
}

bool CommunicationNanotecMotorController::initializeConnection() {
	fileDescriptor = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	if (fileDescriptor == -1) {	/* Error Checking */
		logger->printError("Error in Opening ttyUSB0. Program will be terminated. "
				"Try: sudo chmod a+rw /dev/SocketName.");
		   stopProgram();
	}	else {
		logger->printInfo("ttyUSB0 Opened Successfully");
	}
	return setAttributesForConnection();
}

bool CommunicationNanotecMotorController::setAttributesForConnection() {

	struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fileDescriptor, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	cfsetispeed(&SerialPortSettings, B115200); /* Set Read  Speed as 115200                       */
	cfsetospeed(&SerialPortSettings, B115200); /* Set Write Speed as 115200                       */

	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST; /*No Output Processing*/

	SerialPortSettings.c_cc[VMIN] = 2;
	SerialPortSettings.c_cc[VTIME] = 0;

	if ((tcsetattr(fileDescriptor,TCSANOW,&SerialPortSettings)) != 0) {
		logger->printError("Error in Setting attributes for connection");
		stopProgram();
		return 0;
	}	else {
		return 1;
	}
}

std::string CommunicationNanotecMotorController::sendCommandAndReceiveAnswer(std::string messageToAsk, int commandParameter) {
	std::stringstream temp;
	temp<<messageToAsk<<commandParameter;
	return sendCommandAndReceiveAnswer(temp.str());
}

std::string CommunicationNanotecMotorController::sendCommandAndReceiveAnswer(std::string tempCommand) {
	std::string command = addBeginCharacter(tempCommand);
	sendMessage(command);
	return readMessage(command);
}

void CommunicationNanotecMotorController::sendMessage(std::string tempCommand) {
	std::string command = addEndOfLineCharacter(tempCommand);
	char write_buffer[command.size()];
	strcpy(write_buffer, command.c_str());
	write(fileDescriptor, write_buffer, sizeof(write_buffer));
}

std::string CommunicationNanotecMotorController::readMessage(std::string& command) {
	tcflush(fileDescriptor, TCIFLUSH);
	char read_buffer[maxMessageSize];
	int bytes_read = read(fileDescriptor, &read_buffer, maxMessageSize);
	std::string answer;
	if (bytes_read == -1) {
		answer = strerror(errno);
	}	else {
		answer= std::string(read_buffer, bytes_read);
		filterAnswer(command, answer);
	}
	return answer;
}

void CommunicationNanotecMotorController::performReferenceProcedure() {
	if (isControllerReferenced() == true) {
		logger->printInfo("Motor is referenced");
	}	else {
		logger->printInfo("Performing reference procedure");

		setIOForReferenceSensor();
		setDirectionToDown();
		setModeToReferenceRun();
		setMaximalSpeedInStepPerS(666);
		setAccelerationRampInHerzS(50001);
		changeAccModeToSinusoidal();
		setPhaseCurrent(80);
		sendCommandAndReceiveAnswer("A"); //start motor
		while(!isControllerReferenced()){
			sleep(1);
		}
	}
}

bool CommunicationNanotecMotorController::isControllerReferenced() {
	if (stringToInt(sendCommandAndReceiveAnswer(":is_referenced")) == 1)
		return true;
	else
		return false;
}

std::string CommunicationNanotecMotorController::addBeginCharacter(std::string stringToChange) {
	stringToChange.insert(0, beginCharacter);
	return stringToChange;
}

std::string CommunicationNanotecMotorController::addEndOfLineCharacter(std::string stringToChange) {
	stringToChange.append(endOfLineCharacter);
	return stringToChange;
}

void CommunicationNanotecMotorController::filterAnswer(std::string& command, std::string& answer) {
	answer = answer.substr(command.size()-1, answer.size()-sizeOfEndLineCharacter);
}

void CommunicationNanotecMotorController::startMovement() {
	if (isControllerReferenced()){
		sendCommandAndReceiveAnswer("A");
	}	else {
		logger->printWarning("The motor is not properly referenced or the controller is not ready. No motion will be commanded");
	}
}

void CommunicationNanotecMotorController::setIOForReferenceSensor() {
	sendCommandAndReceiveAnswer("l5154");//("l9250"); //setting limit switch behaviour
	sendCommandAndReceiveAnswer("K20"); //Setting the debounce time for the inputs
	sendCommandAndReceiveAnswer(":port_in_f7"); //set input 6 as sensor for refernce run
	sendCommandAndReceiveAnswer(":port_out_a0");//set output to provide power for sensor
	sendCommandAndReceiveAnswer("Y65536"); //set output to high to power the sensor
}

void CommunicationNanotecMotorController::readErrorCode() {
	std::string errorName = sendCommandAndReceiveAnswer("Z22E");
	int errorCode = stringToInt(sendCommandAndReceiveAnswer("Z23E"));
	std::string errorMsg;
	switch (errorCode) {
	case 1:
		errorMsg ="ERROR_LOWVOLTAGE - undervoltage";
		break;
	case 2:
		errorMsg ="ERROR_TEMP - Temperature of the motor controller is outside of the specified range";
		break;
	case 4:
		errorMsg ="ERROR_TMC - Overcurrent switch-off of the dspDrive was triggered";
		break;
	case 8:
		errorMsg ="ERROR_EE - Incorrect data in the EEPROM, e.g. step resolution is 25th of one step";
		break;
	case 16:
		errorMsg ="ERROR_QEI - Position error";
		break;
	case 32:
		errorMsg ="ERROR_INTERNAL - Internal error (equivalent to the Windows blue screen)";
		break;
	case 128:
		errorMsg ="ERROR_DRIVER - Driver component returned one error";
		break;
	default:
		std::ostringstream strs;
		strs <<"Unknown error occured. Error number:"<<errorCode<<std::endl;
		errorMsg = strs.str();
		break;
	}
	logger->printError(errorMsg);
}

bool CommunicationNanotecMotorController::setPhaseCurrent(int percentValue) {
	if (percentValue > 100){
		logger->printWarning("Phase current value in percent higher than 100 should be avoided. The value will not be changed.");
		return false;
	} else if (percentValue <= 0) {
		logger->printWarning("Phase current value in percent have to be higher than 0.");
		return false;
	}	else {
		sendCommandAndReceiveAnswer("i", percentValue);
		return true;
	}
}

void CommunicationNanotecMotorController::isMotorReferenced() {
	if (isInitialized) {
		int  fl;
		fl = fcntl(fileDescriptor, F_GETFL);
		if (fl == -1) {
			logger->printError("Connection with controller is lost!");
			stopProgram();
		}
	}	else {
		logger->printWarning("Message was not send. You have to initialize the connection first!");
	}
}

void CommunicationNanotecMotorController::changeAccModeToSinusoidal(){
	sendCommandAndReceiveAnswer(":ramp_mode", 1);
}

void CommunicationNanotecMotorController::changeAccModeToTrapeziodal(){
	sendCommandAndReceiveAnswer(":ramp_mode", 0);
}

void CommunicationNanotecMotorController::changeAccModeToJerkFree(){
	sendCommandAndReceiveAnswer(":ramp_mode", 2);
}

void CommunicationNanotecMotorController::setMaximalSpeedInStepPerS(int speedInHertz){
	sendCommandAndReceiveAnswer("o", speedInHertz);
}

int CommunicationNanotecMotorController::getMaximalSpeedInStepPerS(){
	return stringToInt(sendCommandAndReceiveAnswer("Zo"));
}

void CommunicationNanotecMotorController::setDirectionToUp(){
	sendCommandAndReceiveAnswer("d1");
}

void CommunicationNanotecMotorController::setDirectionToDown(){
	sendCommandAndReceiveAnswer("d0");
}

void CommunicationNanotecMotorController::setTravelDistanveInSteps(int travelDistanceInSteps){
	sendCommandAndReceiveAnswer("s", travelDistanceInSteps);
}

void CommunicationNanotecMotorController::setAccelerationRampInHerzS(int maxAccInHzS){
	sendCommandAndReceiveAnswer(":accel", maxAccInHzS);
}

void CommunicationNanotecMotorController::setModeToReferenceRun(){
	sendCommandAndReceiveAnswer("p4");
}

void CommunicationNanotecMotorController::setModeToPositionRelative(){
	sendCommandAndReceiveAnswer("p1");
}

void CommunicationNanotecMotorController::setModeToPositionAbsolute(){
	sendCommandAndReceiveAnswer("p2");
}

void CommunicationNanotecMotorController::setModeToSpeedMode(){
	sendCommandAndReceiveAnswer("p5");
}

void CommunicationNanotecMotorController::setRepetitionsToSingleRun(){
	sendCommandAndReceiveAnswer("W1");
}

int CommunicationNanotecMotorController::getPositionInSteps(){
	return stringToInt(sendCommandAndReceiveAnswer("C"));
}

double CommunicationNanotecMotorController::getStepSize(){
	return 1.0/stringToInt(sendCommandAndReceiveAnswer("Zg"));
}

void CommunicationNanotecMotorController::stopMovementQuick(){
	sendCommandAndReceiveAnswer("S1");
}

void CommunicationNanotecMotorController::stopMovementRamp(){
	sendCommandAndReceiveAnswer("S0");
}

std::string CommunicationNanotecMotorController::askForControllerSeries(){
	return sendCommandAndReceiveAnswer("v");
}


int CommunicationNanotecMotorController::stringToInt(std::string stringValue){
	return atoi(stringValue.c_str());
}

void CommunicationNanotecMotorController::stopProgram(){
	exit(1);
}

void CommunicationNanotecMotorController::setStepSize(int stepSize){
	sendCommandAndReceiveAnswer("g", stepSize);
}

int CommunicationNanotecMotorController::readControllerStatus() {
	std::string statusString = sendCommandAndReceiveAnswer("$");
	//for this function we need to filter out additional two zeros at the beginning. These two zeros do not occur with any other command.
	int status = stringToInt(statusString.substr(2, statusString.size()));
	return status;
}

}
