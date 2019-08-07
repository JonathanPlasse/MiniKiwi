#include"SpeedController.h"
#include<Arduino.h>
#include<PID_v1.h>

SpeedController::SpeedController(DriveMotor mot, Encoder enc, int stepsPerTurn, int sampleTime) : _setPoint(0.0), _Kp(1.0), _Ki(1.0), Kd(1.0){
	this->_mot = mot;
	this-> _enc = enc;
	this->_stepsPerTurn = stepsPerTurn;
	this->_sampleTime = sampleTime;
	this->_posCurrent = _enc.read();
	this->_posOld = _enc.read();

	PID SC(&_input, &_output, &_setPoint, _Kp, _Ki, _Kd, DIRECT);

	this->_max_PWM = 2 << (mot.get_res()-1)
	SC.SetOutputLimits(-_max_PWM, _max_PWM);

	SC.SetMode(AUTOMATIC);
}

void SpeedController::PID_SetVal(float Kp, float Ki, float Kd){
	this->_Kp = Kp;
	this->_Ki = Ki;
	this->_Kd = Kd;
	SC.SetTunings(_Kp, _Ki, _Kd)
}

void SpeedController::SetSpeed(float speed){
	this->_setPoint = (speed*_stepsPerTurn)/_sampleTime;
}

void SpeedController::computeSC(){
	this->_input = (_posCurrent-_posOld);
	this->_posOld = _pos_current;
	this->_posCurrent = _enc.read();

	SC.compute();

	this->mot.set_pwm(_output);
}
