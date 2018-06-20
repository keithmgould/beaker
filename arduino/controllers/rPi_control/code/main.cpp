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
float outerDt = 0; // actual loop time of outer (slower) loop

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

// similar to buildTelemetry. Meant to stay small.
String buildState(){
  String state = String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  state += "," + String(wheels.getTotalPhi(),4) + "," + String(wheels.getPhiDotAvg(),4);
  state += ", " + String(outerDt,2) + "," + String(wheels.getTargetRadsPerSec(),4);
  return state;
}

// similar to buildState. Able to have more fields.
String buildTelemetry(){
  // outer loop time
  String log = String(outerDt);

  // raw states
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getTotalPhi(),4) + "," + String(wheels.getPhiDotAvg(),4);

  // calibrations
  log += "," + String(my_imu.getThetaOffset(),4);

  // final result
  log += "," + String(wheels.getTargetRadsPerSec(),4);

  return log;
}

void sendTelemetry(){
  String log = buildTelemetry();
  Serial3.println(log);
}

void sendState(){
  String state = buildState();
  piTalk.sendToPi(state);
}

void showHelp(){
  String response = "S: send state.\n";
  response += "B: update Theta Offset.\n";
  response += "M: update Motor Pids.\n";
  response += "W: update to given rads/sec.\n";
  response += "A: accelerate by given rads/sec.\n";
  response += "R: reset robot.\n";
  response += "H: this help.\n";

  piTalk.sendToPi(response);
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'S': sendState(); break;
    case 'H': showHelp(); break;
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
    outerDt = outerWaiter.starting();
    my_imu.update();
    // if(my_imu.isEmergency()) { emergencyStop(); }

    // sendTelemetry();
    piTalk.checkForPiCommand();
  }
}
