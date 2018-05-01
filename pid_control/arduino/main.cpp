/*

This little arduino script is to tune and play with the parameters
of the motorControl PID

*/

#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream> // stringstream
#include "../../cpp_lib/constants.h"     // yeah. Constants.
#include "../../cpp_lib/imu.h"
#include "../../cpp_lib/waiter.h"        // waiter helper to help with...waiting
#include "../../cpp_lib/pitalk.h"
#include "../../cpp_lib/pid.h"        // PID library
#include "../../cpp_lib/wheels.h"        // control get raw encoder state
#include "../../cpp_lib/outputs.h"      // bells and whistles (leds, buzzer)

Imu my_imu;
Wheels wheels;
PiTalk piTalk;
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);
Pid thetaPid(POSITION_CONTROL_TIMESTEP);
void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void printStuff(float dt, float newRadPerSec){
  String log = String(dt);
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getPhi(),4) + "," + String(wheels.getPhiDot(),4);
  log += "," + String(my_imu.getThetaOffset());
  log += "," + thetaPid.getParamString();
  log += "," + thetaPid.getTermString();
  log += "," + String(newRadPerSec);
  Serial3.println(log);
}

void updatePidParameters(std::string message){
  float paramVals[3] = {}; // all zeros
  piTalk.stringToFloats(message, paramVals);
  thetaPid.updateParameters(paramVals[0], paramVals[1], paramVals[2]);
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'K': updatePidParameters(message); break;
  }
}

void setup() {
  piTalk.setup(&wheels, &my_imu, &handlePiTalk);
  Serial.begin(115200); while (!Serial) {;}
  Serial3.begin(115200); while (!Serial3) {;}
  Serial.println("\n\nBeginning initializations...");
  my_imu.setup();
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  Serial.println("Setting up interrupts...Done!");
  wheels.initialize();
  Outputs::init();
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
    float dt_ratio = outerDt / (float) POSITION_CONTROL_TIMESTEP;
    my_imu.update(dt_ratio);
    float newRadPerSec = thetaPid.generateCommand(my_imu.getTheta(), outerDt);
    wheels.updateRadsPerSec(newRadPerSec);
    printStuff(outerDt, newRadPerSec);
    piTalk.checkForPiCommand();
  }
}
