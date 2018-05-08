#include "Arduino.h"
#include <Servo.h>
#include "../../cpp_lib/constants.h"
#include "../../cpp_lib/servoMotor.h"

/*
	Stepping up from the pwmMotor system check, this script makes use of the
	ServoMotor class.

	Like pwmMotor system check, this will only work if Beaker is wired and configured
	for PWM commands to motor driver (vs Serial)

	Argument to updatePower ranges from -1 to 1.

	Checks:

	0. Given a positive value, check that the wheels spin forward. 
	0.       (See encoder system check for clarification on "forward")
	0.       check that positive calls to updatePower make the phi/distance values increase.
	1. Given a negative value, check that the wheels spin backward
	1.       check that negative calls to updatePower make the phi/distance values decrease.

*/

ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, LEFT);
ServoMotor motorRight  = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, RIGHT);

void leftEncoderEvent(){ motorLeft.encoderEvent(); }
void rightEncoderEvent(){ motorRight.encoderEvent(); }

void setup(){
	Serial.begin(115200); while (!Serial) {;}
	motorLeft.attach(LEFT_MOTOR_DRIVER);
	motorRight.attach(RIGHT_MOTOR_DRIVER);
	attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  motorLeft.updatePower(0.1); 	// power ranges from -1 to 1
  motorRight.updatePower(0.0);	// power ranges from -1 to 1
}

void loop(){
	String foo = "phiLeft: " + String(motorLeft.getPhi()) + ", xLeft: " + String(motorLeft.getDistance());
	foo += ", phiRight: " + String(motorRight.getPhi()) + ", xRight: " + String(motorRight.getDistance());
	Serial.println(foo);
	delay(500);
}