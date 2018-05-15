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

float thetaDotConstant = 0;
float phiDotConstant = 0;
float recoveringConstant = 0;
float targetAcceleration = 0;

void printStuff(float dt, float accelerationModifier, float phiDotModifier){

  // loop time
  String log = String(dt);

  // raw states
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getX(),4) + "," + String(wheels.getPhiDotAvg(),4);

  // control states
  log += "," + String(targetAcceleration,4);
  log += "," + String(wheels.getTargetRadsPerSec(),4);

  // calibrations
  log += "," + String(my_imu.getThetaOffset(),4);
  log += "," + String(thetaDotConstant,4);
  log += "," + String(recoveringConstant,4);
  log += "," + String(phiDotConstant,4);
  log += "," + String(accelerationModifier,4);
  log += "," + String(phiDotModifier,4);

  Serial3.println(log);
}

void zeroAllParameters(bool piTalkResponse = true){
  thetaDotConstant = phiDotConstant = targetAcceleration = 0;
  wheels.updateRadsPerSec(0);
  if(piTalkResponse) { piTalk.sendToPi("Zeroed Out"); }
}

void updateRecoveringConstant(std::string message){
  recoveringConstant = String(message.c_str()).toFloat();

  piTalk.sendToPi("Updated Recovering Constant.");
}

void updateThetaDotConstant(std::string message){
  thetaDotConstant = String(message.c_str()).toFloat();

  piTalk.sendToPi("Updated thetaDot Constant.");
}

void updatePhiDotConstant(std::string message){
  phiDotConstant = String(message.c_str()).toFloat();

  piTalk.sendToPi("Updated phiDot Constant.");
}

void showHelp(){
  String response = "T: update thetaDot constant.\n";
  response += "P: update phiDot constant.\n";
  response += "R: update recovering constant.\n";
  response += "Z: zero all parameters.\n";
  response += "H: this help\n";

  piTalk.sendToPi(response);
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'T': updateThetaDotConstant(message); break;
    case 'P': updatePhiDotConstant(message); break;
    case 'R': updateRecoveringConstant(message); break;
    case 'Z': zeroAllParameters(); break;
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

    float theta = my_imu.getTheta();
    float thetaDot = my_imu.getThetaDot();

    float accelerationModifier = fabs(theta * thetaDot * thetaDotConstant);

    if(theta > 0 && thetaDot > 0){ // if leaning forward and accelerating forward (BAD)
      targetAcceleration += accelerationModifier; // accelerate forward
    } else if(theta < 0 && thetaDot < 0){ // if leaning backward and accelerating backward (BAD)
      targetAcceleration -= accelerationModifier;  // accelerate backward
    } else if(theta > 0 && thetaDot < 0 ) {  // if leaning forward and accelerating backward (GOOD)
      if(fabs(thetaDot) > recoveringConstant * theta){
        targetAcceleration = 0;
      }else{
        targetAcceleration += accelerationModifier;
      }
    } else if(theta < 0 && thetaDot > 0) {  // if leaning backward and accelerating forward (GOOD)
      if(thetaDot > recoveringConstant * fabs(theta)){
        targetAcceleration = 0;
      }else{
        targetAcceleration -= accelerationModifier;
      }
    }

    float phiDot = wheels.getPhiDotAvg();
    float phiDotModifier;
    if (fabs(theta) < 0.05){
      phiDotModifier = phiDot * phiDotConstant;
    }else{
      phiDotModifier = 0;
    }

    targetAcceleration += phiDotModifier;

    float newRadsPerSec = wheels.getTargetRadsPerSec() + targetAcceleration;
    newRadsPerSec = constrain(newRadsPerSec,-15,15);
    wheels.updateRadsPerSec(newRadsPerSec);
    

    printStuff(outerDt, accelerationModifier, phiDotModifier);
    piTalk.checkForPiCommand();
  }
}
