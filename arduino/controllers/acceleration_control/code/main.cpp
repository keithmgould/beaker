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

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

float thetaDotConstant = 0;
float phiDotConstant = 0;
float angleRateRatioConstant = 0;
float angleResponseConstant = 0;

void printStuff(float dt, float risingAngleOffset){

  // loop time
  String log = String(dt);

  // raw states
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getX(),4) + "," + String(wheels.getPhiDotAvg(),4);

  // control states
  log += "," + String(risingAngleOffset,4);
  log += "," + String(wheels.getTargetRadsPerSec(),4);

  // calibrations
  log += "," + String(my_imu.getThetaOffset(),4);
  log += "," + String(angleRateRatioConstant,4);
  log += "," + String(angleResponseConstant,4);


  Serial3.println(log);
}

void zeroAllParameters(bool piTalkResponse = true){
  angleRateRatioConstant = angleResponseConstant = 0;
  wheels.updateRadsPerSec(0);
  if(piTalkResponse) { piTalk.sendToPi("Zeroed Out"); }
}

void updateAngleRateRatioConstant(std::string message){
  angleRateRatioConstant = String(message.c_str()).toFloat();

  piTalk.sendToPi("Updated angleRateRatioConstant.");
}

void updateAngleResponseConstant(std::string message){
  angleResponseConstant = String(message.c_str()).toFloat();

  piTalk.sendToPi("Updated angleResponseConstant Constant.");
}

void showHelp(){
  String response = "A: update angleRateRatioConstant.\n";
  response += "R: update angleResponseConstant.\n";
  response += "Z: zero all parameters.\n";
  response += "H: this help\n";

  piTalk.sendToPi(response);
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'A': updateAngleRateRatioConstant(message); break;
    case 'R': updateAngleResponseConstant(message); break;
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
  piTalk.setup(&wheels, &my_imu, &handlePiTalk);
  Serial.begin(115200); while (!Serial) {;}  // primary serial
  Serial3.begin(115200); while (!Serial3) {;} // bluetooth serial
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
    
    float risingAngleOffset = thetaDot * angleRateRatioConstant + theta;

    float newRadsPerSec = wheels.getTargetRadsPerSec() + angleResponseConstant * risingAngleOffset;

    newRadsPerSec = constrain(newRadsPerSec, -10,10);
    wheels.updateRadsPerSec(newRadsPerSec);
    printStuff(outerDt, risingAngleOffset);
    piTalk.checkForPiCommand();
  }
}
