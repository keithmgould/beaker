#include "Arduino.h"
#include <Servo.h>
#include "../../cpp_lib/constants.h"
#include "../../cpp_lib/servoMotor.h"

/*
	Stepping up from the pwmMotor system check, this script makes use of the
	ServoMotor class.

	Like pwmMotor system check, this will only work if Beaker is wired and configured
	for PWM commands to motor driver (vs Serial)

	Checks:

	0. that the wheels spin (duh).
	1. that positive calls to updatePower make the phi/distance values increase.
	2. that negative calls to updatePower make the phu/distance values decrease.
*/

ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, FORWARD);
ServoMotor motorRight  = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, BACKWARD);

void leftEncoderEvent(){ motorLeft.encoderEvent(); }
void rightEncoderEvent(){ motorRight.encoderEvent(); }

void setup(){
	Serial.begin(115200); while (!Serial) {;}
	motorLeft.attach(LEFT_MOTOR_DRIVER);
	motorRight.attach(RIGHT_MOTOR_DRIVER);
	attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  motorLeft.updatePower(0.1); 	// power ranges from -1 to 1
  motorRight.updatePower(0.1);	// power ranges from -1 to 1
}

void loop(){
	String foo = "phiLeft: " + String(motorLeft.getPhi()) + ", xLeft: " + String(motorLeft.getDistance());
	foo += ", phiRight: " + String(motorRight.getPhi()) + ", xRight: " + String(motorRight.getDistance());
	Serial.println(foo);
	delay(500);
}