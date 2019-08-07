#idndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include"../drive_motor/DriveMotor.h"
#include <Encoder.h>

class SpeedController{
public:
	//Set up speed control
	SpeedController(DriveMotor mot, Encoder enc, int stepsPerTurn, int sampleTime);

	//Sets PID Values
	void PID_SetVal(float Kp, float Ki, float Kd);

	//Sends speed command in rpm
	void setSpeed(float speed);

	//Computes new PWM value
	void computeSC();

	//Prints current speed to serial port in rpm
	void printSpeed();

	/*
	//Pauses the PID loop, drives the motor to 0 rpm
	void pause();

	//Resumes the PID loop, drives the motor to resumeSpeed rpm
	void resume(float resumeSpeed);

	//Stops the PID loop and motor
	void stop();
	*/

private:
	float _Kp, _Ki, _Kd, _input, _output, _setPoint;
	int sampleTime, _stepsPerTurn, _posCurrent, _posOld, _max_PWM;
	Encoder _enc;
	DriveMotor _mot;

}

#endif
