#include "Arduino.h"
#include <Servo.h>
#include "../../cpp_lib/constants.h"
#include "../../cpp_lib/serialMotor.h"

/*
	Stepping up from the rawSerialMotor system check, this script makes use of the
	SerialMotor class.

	Like rawSerialMotor system check, this will only work if Beaker is wired and configured
	for Serial commands to motor driver (vs PWM)

	Argument to updatePower ranges from -1 to 1.

	NOTE: This test is meant to be performed with Beaker hanging, wheels NOT touching the ground.

	Checks:

	0. Given a positive value, check that the wheels spin forward. 
	0.       (See encoder system check for clarification on "forward")
	0.       check that positive calls to updatePower make the phi/distance values increase.
	1. Given a negative value, check that the wheels spin backward
	1.       check that negative calls to updatePower make the phi/distance values decrease.

*/

SerialMotor motorLeft  = SerialMotor(LH_ENCODER_A, LH_ENCODER_B, LEFT);
SerialMotor motorRight  = SerialMotor(RH_ENCODER_A, RH_ENCODER_B, RIGHT);

void leftEncoderEvent(){ motorLeft.encoderEvent(); }
void rightEncoderEvent(){ motorRight.encoderEvent(); }

void setup(){
	Serial1.begin(9600); while (!Serial) {delay(1);}
	Serial.begin(115200); while (!Serial) {delay(1);}
	attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  motorLeft.updatePower(0.2); 	// power ranges from -1 to 1
  motorRight.updatePower(0.2);	// power ranges from -1 to 1
}

void loop(){
	String foo = "phiLeft: " + String(motorLeft.getPhi()) + ", xLeft: " + String(motorLeft.getDistance());
	foo += ", phiRight: " + String(motorRight.getPhi()) + ", xRight: " + String(motorRight.getDistance());
	Serial.println(foo);
	delay(500);
}