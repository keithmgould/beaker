#include <Servo.h>
#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                      // stringstream
#include "../../cpp_lib/includes.h"     // common Beaker functionality

Wheels wheels;
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void printStuff(int outerDt){
	String str = String(outerDt) + ",";
	str += String(wheels.getLeftPhiDot()) + ",";
	str += String(wheels.getRightPhiDot()) + ",";
	Serial.println(str);
}

void setup(){
	Serial.begin(115200); while (!Serial) {;}
	attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  wheels.initialize();
  wheels.updateRadsPerSec(0.01);
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
    printStuff(outerDt);
  }
}