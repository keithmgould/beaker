#include <Servo.h>
#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                      // stringstream
#include "../../cpp_lib/includes.h"     // common Beaker functionality

/*
    Find out more about the wheels object in the comments of the wheels.h file.

    In this system check, check for the following:

    0. Wheels both spin in same direction
    1. Using a tachometer, make sure the commanded rads/sec is actually happening.
    2. Make sure the printout of rads/sec matches expected rads/sec, and also matches tachometer.
    3. Make sure direction between expected rads/sec and printed rads/sec matches
*/

Wheels wheels;
PiTalk piTalk;

Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void printStuff(int outerDt){
	String str = String(outerDt) + ",";
  str += "setpoint: " + String(wheels.getLeftSetpoint());
	str += ", leftPhiDot: " + String(wheels.getLeftPhiDot(),4);
  str += ", leftPhiDelta: " + String(wheels.getLeftPhiDelta(),4);
  str += ", leftCommand: " + String(wheels.getLeftCommand(),4);
  str += ", left params PID: " + wheels.getLeftPidParams();
	Serial.println(str);
}

void setup(){
  piTalk.setup(&wheels);
	Serial.begin(115200); while (!Serial) {;}
	attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  wheels.initialize();
  wheels.updateRadsPerSec(0.0);
}

void loop(){
	// inner loop behavior
	if(innerWaiter.isTime()){
		float innerDt = innerWaiter.starting();
    wheels.spin(innerDt);
  }

  // outer loop behavior
  if(outerWaiter.isTime()){
    float outerDt = outerWaiter.starting();
    piTalk.checkForPiCommand();
    // printStuff(outerDt);
  }
}