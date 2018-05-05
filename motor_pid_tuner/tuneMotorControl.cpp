/*

This little arduino script is to tune and play with the parameters
of the motorControl PID

*/

#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream> // stringstream
#include "../cpp_lib/includes.h"     // yeah. Constants.

Wheels wheels;
Imu my_imu;
PiTalk piTalk;

Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);
void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

// Todo: update Pitalk so this is not necessary
void handlePiTalk(char command, std::string message){}

void setup() {
  piTalk.setup(&wheels, &my_imu, &handlePiTalk);
  my_imu.setup();
  Serial.begin(115200); while (!Serial) {;}
  wheels.initialize();

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
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
    my_imu.update();
    float newRadPerSec = thetaPid.generateCommand(my_imu.getTheta(), outerDt);
    wheels.updateRadsPerSec(newRadPerSec);
    piTalk.checkForPiCommand();
  }
}
