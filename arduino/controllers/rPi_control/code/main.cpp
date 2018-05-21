#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <EEPROM.h>
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                      // stringstream
#include "../../../lib/includes.h"     // common Beaker functionality

Imu my_imu;
Wheels wheels;
PiTalk piTalk;
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);
String state;

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void buildState(){
  state = String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  state += "," + String(wheels.getX(),4) + "," + String(wheels.getPhiDotAvg(),4);  
}

void sendTelemetry(float dt, float newRadPerSec){
  String log = String(dt);

  // raw states
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getX(),4) + "," + String(wheels.getPhiDotAvg(),4);

  // calibrations
  log += "," + String(my_imu.getThetaOffset(),4);

  // final result
  log += "," + String(newRadPerSec,4);

  Serial3.println(log);
}

void sendState(){
  buildState();
  piTalk.sendToPi(state);
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'S': sendState(); break;
  }
}

void emergencyStop(){
  wheels.updateRadsPerSec(0);
  Outputs::beep(100,100);
  Outputs::beep(100,100);
  Outputs::beep(100,100);
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
    if(my_imu.isEmergency()) { emergencyStop(); }

    piTalk.checkForPiCommand();
  }
}
