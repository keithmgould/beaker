#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <EEPROM.h>
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                      // stringstream
#include "../../../cpp_lib/includes.h"     // common Beaker functionality

Pid thetaPid(POSITION_CONTROL_TIMESTEP);
Pid thetaDotPid(POSITION_CONTROL_TIMESTEP);
Pid xPosPid(POSITION_CONTROL_TIMESTEP);
Pid phiDotPid(POSITION_CONTROL_TIMESTEP);

Imu my_imu;
Wheels wheels;
PiTalk piTalk;
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void printStuff(float dt, float newRadPerSec, float phiDotAvg, float thetaTerm, float thetaDotTerm, float xPosTerm, float phiDotTerm){
  String log = String(dt);

  // raw states
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getX(),4) + "," + String(phiDotAvg,4);

  // calibrations
  log += "," + String(my_imu.getThetaOffset());

  // final result
  log += "," + String(newRadPerSec);
  
  // PID term components
  log += "," + String(thetaPid.getTermString());
  log += "," + String(thetaDotPid.getTermString());
  log += "," + String(xPosPid.getTermString());
  log += "," + String(phiDotPid.getTermString());

  // PID final term  
  log += "," + String(thetaTerm);
  log += "," + String(thetaDotTerm);
  log += "," + String(xPosTerm);
  log += "," + String(phiDotTerm);

  Serial3.println(log);
}

void zeroAllParameters(){
  thetaPid.updateParameters(0,0,0);
  thetaDotPid.updateParameters(0,0,0);
  xPosPid.updateParameters(0,0,0);
  phiDotPid.updateParameters(0,0,0);
}

void storeAllPidParameters(){
  unsigned int addr = 0;
  addr = thetaPid.storeParameters(addr);
  addr = thetaDotPid.storeParameters(addr);
  addr = xPosPid.storeParameters(addr);
  phiDotPid.storeParameters(addr);
}

void loadAllPidParameters(){
  unsigned int addr = 0;
  addr = thetaPid.loadParameters(addr);
  addr = thetaDotPid.loadParameters(addr);
  addr = xPosPid.loadParameters(addr);
  phiDotPid.loadParameters(addr);
}

void showPidValues(){
  piTalk.sendToPi('H', thetaPid.getParamString());
}

void updatePidParameters(int component, std::string message){
  float paramVals[3] = {}; // all zeros
  piTalk.stringToFloats(message, paramVals);
  switch(component){
    case 0:
      thetaPid.updateParameters(paramVals[0], paramVals[1], paramVals[2]);
      break;
    case 1:
      thetaDotPid.updateParameters(paramVals[0], paramVals[1], paramVals[2]);
      break;
    case 2:
      xPosPid.updateParameters(paramVals[0], paramVals[1], paramVals[2]);
      break;
    case 3:
      phiDotPid.updateParameters(paramVals[0], paramVals[1], paramVals[2]);
      break;
  }
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'T': updatePidParameters(0, message); break;
    case 'D': updatePidParameters(1, message); break;
    case 'X': updatePidParameters(2, message); break;
    case 'P': updatePidParameters(3, message); break;
    case 'S': storeAllPidParameters(); break;
    case 'L': loadAllPidParameters(); break;
    case 'H': showPidValues(); break;
    case 'Z': zeroAllParameters(); break;
  }
}

void setup() {
  // thetaPid.setClearAccumulatorWhenCrossingZero(true);

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

    float thetaTerm = thetaPid.generateCommand(my_imu.getTheta(), outerDt);
    float thetaDotTerm = thetaDotPid.generateCommand(my_imu.getThetaDot(), outerDt);
    float xPosTerm = xPosPid.generateCommand(wheels.getX(), outerDt);
    float phiDotTerm = phiDotPid.generateCommand(phiDotAvg, outerDt);

    float newRadPerSec = thetaTerm + thetaDotTerm + xPosTerm + phiDotTerm;
    
    // discuss
    newRadPerSec = -newRadPerSec;

    wheels.updateRadsPerSec(newRadPerSec);
    printStuff(outerDt, newRadPerSec, phiDotAvg, thetaTerm, thetaDotTerm, xPosTerm, phiDotTerm);
    piTalk.checkForPiCommand();
  }
}
