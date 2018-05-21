#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <EEPROM.h>
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                      // stringstream
#include "../../../lib/includes.h"     // common Beaker functionality
#include "./lib/p4.h"

P4 p4;
Imu my_imu;
Wheels wheels;
PiTalk piTalk;
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void printStuff(float dt, float newRadPerSec, float phiDotAvg){
  String log = String(dt);
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getX(),4) + "," + String(phiDotAvg,4);
  log += "," + String(my_imu.getThetaOffset());
  log += "," + p4.getParamString();
  log += "," + p4.getTermString();
  log += "," + String(newRadPerSec);
  Serial3.println(log);
}

void updateP4Parameters(std::string message){
  float paramVals[4] = {}; // all zeros
  piTalk.stringToFloats(message, paramVals);
  p4.updateParameters(paramVals[0], paramVals[1], paramVals[2], paramVals[3]);
}

void zeroP4Parameters(std::string message){
 p4.updateParameters(0,0,0,0); 
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'K': updateP4Parameters(message); break;
    case 'Z': zeroP4Parameters(message); break;
  }
}

void setup() {
  piTalk.setup(&wheels, &my_imu, &handlePiTalk);
  Serial.begin(115200); while (!Serial) {;}
  Serial3.begin(115200); while (!Serial3) {;} // bluetooth
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
    my_imu.pushThetaDotData();
  }

  // outer loop behavior
  if(outerWaiter.isTime()){
    float outerDt = outerWaiter.starting();
    my_imu.update();
    float phiDotAvg = wheels.getPhiDotAvg();
    float newRadPerSec = p4.computeNewRadsPerSec(my_imu.getTheta(), my_imu.getThetaDot(), wheels.getX(), phiDotAvg);
    wheels.updateRadsPerSec(newRadPerSec);
    printStuff(outerDt, newRadPerSec, phiDotAvg);
    piTalk.checkForPiCommand();
  }
}
