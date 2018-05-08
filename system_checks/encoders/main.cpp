#include "Arduino.h"
#include <Servo.h>
#include "../../cpp_lib/constants.h"
#include "../../cpp_lib/servoMotor.h"

/*

	This check is just for the motor encoders.

  Each motor is marked as L(eft) amnd R(right).
	
	These designations also yield which direction is "forward" for Beaker.

	Given the notion of "forward," spinning a wheel in the direction
	that moves Beaker forward is the same as spinning the wheel forward.

	Checks:

	0. manually rotate the left wheel FORWARD . Make sure phiLeft and xLeft INCREASE.
	0.     Make sure phi resets to 0 after a full rotation (2 * PI) (Make sure phi is never negative)
	0.     Make sure xLeft continually increases.
	1. manually rotate the left wheel BACKWARD. Make sure phiLeft and xLeft DECREASE.
	1.     Make sure phi jumpts back to to (2*PI) after it reaches zero.
	1.     Make sure xLeft continually decresases into negative numbers.
  2. Do the same thing with the right wheel.

*/

ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, LEFT);
ServoMotor motorRight  = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, RIGHT);

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