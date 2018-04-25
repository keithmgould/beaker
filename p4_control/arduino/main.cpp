#include <Adafruit_Sensor.h>               // IMU
#include <Adafruit_BNO055.h>               // IMU
#include <Wire.h>                          // I2C for IMU
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                         // stringstream
#include "../../cpp_lib/constants.h"     // These apply regardless of control method
#include "../../cpp_lib/imu.h"           // class to wrap IMU, which holds theta and thetaDot
#include "../../cpp_lib/waiter.h"        // waiter helper to help with...waiting
#include "../../cpp_lib/wheels.h"        // control get raw encoder state
#include "../../cpp_lib/p4.h"            // P4 Control Algorithm

P4 p4;
Imu my_imu;
Wheels wheels;
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void printStuff(float dt){
  String log = String(dt);
  log += "," + String(my_imu.getTheta(),4) + "," + String(my_imu.getThetaDot(),4);
  log += "," + String(wheels.getPhi(),4) + "," + String(wheels.getPhiDot(),4);
  log += p4.getKString();
  Serial.println(log);
}

void setup() {
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
  }
}
