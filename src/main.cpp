#include <Arduino.h>
#include <Encoder.h>
#include "../include/miniKiwiPins.h"
#include "../lib/drive_motor/DriveMotor.h"
#include "../lib/motor_control/spedd_controller/SpeedController.h"

DriveMotor motor1(IN3, IN4);
uint8_t res = 0;
uint16_t max_pwm = 0;

void setup() {
	delay(5000);
	Serial.begin(9600);
	Serial.println("Starting PID Test");
	pinMode(DEBUG, OUTPUT);
	digitalWrite(DEBUG, HIGH);

}

void loop() {


}
