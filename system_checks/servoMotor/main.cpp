#include "Arduino.h"
#include <Servo.h>
#include "../../cpp_lib/constants.h"
#include "../../cpp_lib/servoMotor.h"

/*
	Stepping up from the pwmMotor system check, this script makes use of the
	ServoMotor class.

	Like pwmMotor system check, this will only work if Beaker is wired and configured
	for PWM commands to motor driver (vs Serial)
*/

ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, 1);
ServoMotor motorRight  = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, -1);

void leftEncoderEvent(){ motorLeft.encoderEvent(); }
void rightEncoderEvent(){ motorRight.encoderEvent(); }

void setup(){
	Serial.begin(115200); while (!Serial) {;}
	motorLeft.attach(LEFT_MOTOR_DRIVER);
	motorRight.attach(RIGHT_MOTOR_DRIVER);
	attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  motorLeft.updatePower(0.1);
  motorRight.updatePower(0.1);
}

void loop(){
	Serial.println(motorLeft.getPhi());
	delay(500);
}