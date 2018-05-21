#include <Servo.h>
#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                      // stringstream
#include "../../lib/includes.h"     // common Beaker functionality

/*
    Find out more about the wheels object in the comments of the wheels.h file.

    Use this check to validate (and tweak) the default PID values of the 
    motor controller.

    The default values are held in constants.h as kP, kI, kD.

    Use the main communication.py and send down the command 'R<rad/sec>'
    which will make Beaker run for 2 seconds with designated rads/sec,
    first resetting itself to capture data. So the flow is modify PID values 
    with the 'MP I D' command, then 'R6.28', etc....

    Beaker is meant to be On The Ground in his Stroller (training wheels),
    which keeps Beaker mostly upright.
*/

Wheels wheels;
PiTalk piTalk;

Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

long second_timer;
int total_time;

long test_time = 2000;

void printStuff(int outerDt){
	String str = String(outerDt);
  str += "," + String(total_time);
  str += "," + String(wheels.getLeftSetpoint());
	str += "," + String(wheels.getPhiDot(),4);
	Serial3.println(str);  // note bluetooth serial port
}

void runTest(std::string message){
  float newRadsPerSec = String(message.c_str()).toFloat();
  wheels.updateRadsPerSec(newRadsPerSec);
  delay(100);
  Serial3.println("\n\nKP,KI,KD");
  Serial3.println(wheels.getLeftPidParams());
  Serial3.println("\n\n");
  delay(100);
  total_time = 0;
  second_timer = millis();
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'R': runTest(message); break;
  }
}

void setup(){
  Serial3.begin(115200); while(!Serial3) {delay(1);} // note bluetooth serial port
  piTalk.setup(&wheels, &handlePiTalk);
	attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  wheels.initialize();
  total_time = 0;
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
    total_time += POSITION_CONTROL_TIMESTEP;
    piTalk.checkForPiCommand();
    if((millis() - second_timer) > test_time){
      wheels.updateRadsPerSec(0);
      total_time = 0;
    }else{
      printStuff(outerDt);
    }
  }
}