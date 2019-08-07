#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>
#include "../include/miniKiwiPins.h"
#include "../lib/drive_motor/DriveMotor.h"

uint8_t res = 0;
uint16_t max_pwm = 0;

const uint32_t sample_time = 5;
const uint32_t time;

DriveMotor motor1(IN4, IN3);
Encoder enc(ENC_A1, ENC_B1);

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,6,0,0, DIRECT);

void setup() {
	delay(5000);
	Serial.begin(9600);
	Serial.println("Starting PID Test");
	pinMode(DEBUG, OUTPUT);
	digitalWrite(DEBUG, HIGH);

	res = motor1.get_res();
	max_pwm = 2<<(res-1);
	Serial.print("Max PWM value calculated : +-");
	Serial.println(max_pwm);

	myPID.SetOutputLimits(-max_pwm, max_pwm);
	myPID.SetSampleTime(sample_time);

	Input = enc.read();
	Setpoint = 1200;

	myPID.SetMode(AUTOMATIC);
	
	time = millis();
}

void loop() {
	Input = enc.read();
	myPID.Compute();
	motor1.set_pwm(int16_t(Output));
	if (millis() - time > sample_time) {
		time += sample_time;
		Serial.print(enc.read());
		Serial.print(" ");
		Serial.println(Output);
	}
}
