#include "Arduino.h"
#include <Servo.h>
#include "../../cpp_lib/constants.h"
#include "../../cpp_lib/servoMotor.h"

/*
	Spin the wheels. Make sure the phi and x position work properly.
*/

ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, 1);
ServoMotor motorRight  = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, -1);

void leftEncoderEvent(){ motorLeft.encoderEvent(); }
void rightEncoderEvent(){ motorRight.encoderEvent(); }

void setup(){
	Serial.begin(115200); while (!Serial) {;}
	attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
}

void loop(){
	String foo = "phiLeft: " + String(motorLeft.getPhi()) + ", xLeft: " + String(motorLeft.getDistance());
	foo += ", phiRight: " + String(motorRight.getPhi()) + ", xRight: " + String(motorRight.getDistance());
	Serial.println(foo);
	delay(500);
}