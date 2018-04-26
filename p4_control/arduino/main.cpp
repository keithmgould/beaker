#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                      // stringstream
#include "../../cpp_lib/constants.h"    // These apply regardless of control method
#include "../../cpp_lib/imu.h"          // class to wrap IMU, which holds theta and thetaDot
#include "../../cpp_lib/waiter.h"       // waiter helper to help with...waiting
#include "../../cpp_lib/wheels.h"       // control get raw encoder state
#include "../../cpp_lib/p4.h"           // P4 Control Algorithm
#include "../../cpp_lib/pitalk.h"       // communication with Raspberry Pi

P4 p4;
Imu my_imu;
Wheels wheels;
PiTalk piTalk;
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void printStuff(float dt){
  String log = String(dt);
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getPhi(),4) + "," + String(wheels.getPhiDot(),4);
  log += "," + String(my_imu.getThetaOffset());
  log += "," + p4.getKString();
  Serial.println(log);
}

void updateP4Parameters(std::string message){
  Serial.println("Hi from updateP4Parameters!!");
  Serial.println(message.c_str());

  float values[4];
  std::istringstream ss( message );
  std::copy(
    std::istream_iterator <float> ( ss ),
    std::istream_iterator <float> (),
    values
    );

  Serial.println("post conversion");

  p4.updateParameters(values[0], values[1], values[2], values[3]);
}

void handlePiTalk(char command, std::string message){
  Serial.println("Hi From Da Callback!");

  switch(command){
    case 'K': updateP4Parameters(message); break;
  }

  Serial.println("All done in handlePiTalk");
}

void setup() {
  piTalk.setup(&wheels, &my_imu, &handlePiTalk);
  Serial.begin(115200); while (!Serial) {;}
  Serial.println("\n\nBeginning initializations...");
  my_imu.setup();
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  Serial.println("Setting up interrupts...Done!");
  wheels.initialize();
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
    float newRadPerSec = p4.computeNewRadsPerSec(my_imu.getTheta(), my_imu.getThetaDot(), wheels.getPhi(), wheels.getPhiDot());
    wheels.updateRadsPerSec(newRadPerSec);
    printStuff(outerDt);
    piTalk.checkForPiCommand();
  }
}
