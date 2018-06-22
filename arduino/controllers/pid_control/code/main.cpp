#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <EEPROM.h>
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                      // stringstream
#include "../../../lib/includes.h"     // common Beaker functionality

Pid thetaPid(POSITION_CONTROL_TIMESTEP);
Pid thetaDotPid(POSITION_CONTROL_TIMESTEP);
Pid xPosPid(POSITION_CONTROL_TIMESTEP);
Pid phiDotPid(POSITION_CONTROL_TIMESTEP);

Imu my_imu;
Wheels wheels;
PiTalk piTalk;
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);
bool motorsOn = true;
float momentumConstant = 0;

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void printStuff(float dt, float newRadPerSec, float radPerSecDelta, float phiDotAvg, float thetaTerm, float thetaDotTerm, float xPosTerm, float phiDotTerm){
  String log = String(dt);

  // raw states
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getTotalPhi(),4) + "," + String(phiDotAvg,4);

  // calibrations
  log += "," + String(my_imu.getThetaOffset(),4);

  // final result
  log += "," + String(newRadPerSec,4);
  
  // PID term components
  log += "," + String(thetaPid.getTermString());
  log += "," + String(thetaDotPid.getTermString());
  log += "," + String(xPosPid.getTermString());
  log += "," + String(phiDotPid.getTermString());

  // PID final term  
  log += "," + String(thetaTerm,4);
  log += "," + String(thetaDotTerm,4);
  log += "," + String(xPosTerm,4);
  log += "," + String(phiDotTerm,4);

  // Extra
  log += "," + String(wheels.getPhiDot(),4);
  log += "," + String(radPerSecDelta,4);
  log += "," + String(momentumConstant,4);

  Serial3.println(log);
}

// Local memory only. Not EEPROM
void zeroAllParameters(bool tellPi){
  thetaPid.updateParameters(0,0,0);
  thetaDotPid.updateParameters(0,0,0);
  xPosPid.updateParameters(0,0,0);
  phiDotPid.updateParameters(0,0,0);
  wheels.updateRadsPerSec(0);
  momentumConstant = 0;

  if(tellPi){ piTalk.sendToPi("Values Zeroed."); }
}

// in EEPROM
void storeAllParameters(){
  unsigned int addr = 0;
  addr = thetaPid.storeParameters(addr);
  addr = thetaDotPid.storeParameters(addr);
  addr = xPosPid.storeParameters(addr);
  addr = phiDotPid.storeParameters(addr);
  addr = EepromHelper::storeFloat(momentumConstant, addr);

  piTalk.sendToPi("Values Saved to EEPROM.");
}

// from EEPROM
void loadAllParameters(){
  unsigned int addr = 0;
  addr = thetaPid.loadParameters(addr);
  addr = thetaDotPid.loadParameters(addr);
  addr = xPosPid.loadParameters(addr);
  addr = phiDotPid.loadParameters(addr);
  EepromHelper::loadFloat(momentumConstant, addr);

  piTalk.sendToPi("Values Loaded from EEPROM.");
}

void showParamValues(){
  String response = "(T)heta: " + thetaPid.getParamString() + "\n";
  response += "theta(D)ot: " + thetaDotPid.getParamString() + "\n";
  response += "(X)Pos: " + xPosPid.getParamString() + "\n";
  response += "(P)hiDot: " + phiDotPid.getParamString() + "\n";
  response += "MotorsOn: " + String(motorsOn) + "\n";
  response += "MomentumConstant: " + String(momentumConstant) + "\n";

  piTalk.sendToPi(response);
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

  piTalk.sendToPi("Updated PID Parameters.");
}

void turnMotorsOn(){ motorsOn = true; piTalk.sendToPi("Motors Turned ON."); }
void turnMotorsOff(){ motorsOn = false; piTalk.sendToPi("Motors Turned OFF."); }

void updateMomentumConstant(std::string message){
  float newMC = String(message.c_str()).toFloat();
  momentumConstant = newMC;
  piTalk.sendToPi("Updated Momentum Constant.");
}

void showHelp(){
  String response = "T: update theta PID terms. Ex: T0.5 -2.4 8.2\n";
  response += "D: update thetaDot PID terms. Ex: D0.5 -2.4 8.2\n";
  response += "X: update xPos PID terms. Ex: X0.5 -2.4 8.2\n";
  response += "P: update phiDot PID terms. Ex: P0.5 -2.4 8.2\n";
  response += "A: update momentumConstant. Ex: A0.5\n";
  response += "L: load PID values from EEPROM\n";
  response += "S: save PID values to EEPROM\n";
  response += "V: show currently used Param Values\n";
  response += "Z: zero out all PID Values.\n";
  response += "J: enable motors. (Pay attention to motor commands)\n";
  response += "K: disable motors. (Ignore motor commands)\n";
  response += "H: this help\n";

  piTalk.sendToPi(response);
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'T': updatePidParameters(0, message); break;
    case 'D': updatePidParameters(1, message); break;
    case 'X': updatePidParameters(2, message); break;
    case 'P': updatePidParameters(3, message); break;
    case 'A': updateMomentumConstant(message); break;
    case 'S': storeAllParameters(); break;
    case 'L': loadAllParameters(); break;
    case 'V': showParamValues(); break;
    case 'Z': zeroAllParameters(true); break;
    case 'J': turnMotorsOn(); break;
    case 'K': turnMotorsOff(); break;
    case 'H': showHelp(); break;
  }
}

void emergencyStop(){
  wheels.updateRadsPerSec(0);
  zeroAllParameters(false);
  Outputs::beep(100,100);
  Outputs::beep(100,100);
  Outputs::beep(100,100);
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
    if(my_imu.isEmergency()) { emergencyStop(); }

    float phiDotAvg = wheels.getPhiDotAvg();
    float xPos = wheels.getX();
    float theta = my_imu.getTheta();

    float thetaTerm = thetaPid.generateCommand(theta);
    float thetaDotTerm = thetaDotPid.generateCommand(my_imu.getThetaDot());
    float xPosTerm = xPosPid.generateCommand(wheels.getX());
    float phiDotTerm = phiDotPid.generateCommand(phiDotAvg);

    // If the xPosTerm did its job and got us leaning back towards X=0, 
    // then stop trying to accelerate away from X=0.
    float momentum = momentumConstant * fabs(phiDotAvg);

    if(xPos > 0 && theta < -momentum){ xPosTerm = 0; }
    if(xPos < 0 && theta > momentum){ xPosTerm = 0; }

    float radPerSecDelta = thetaTerm + thetaDotTerm + xPosTerm + phiDotTerm;
  
    radPerSecDelta = -radPerSecDelta;

    float newRadPerSec = wheels.getTargetRadsPerSec();

    newRadPerSec += radPerSecDelta;

    newRadPerSec = constrain(newRadPerSec, -10,10);

    if(motorsOn){
      wheels.updateRadsPerSec(newRadPerSec);  
    }else{
      wheels.updateRadsPerSec(0);
    }

    printStuff(outerDt, newRadPerSec, radPerSecDelta, phiDotAvg, thetaTerm, thetaDotTerm, xPosTerm, phiDotTerm);
    piTalk.checkForPiCommand();
  }
}
