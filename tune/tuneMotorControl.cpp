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
#include "../cpp_lib/constants.cpp"     // yeah. Constants.
#include "../cpp_lib/waiter.cpp"        // waiter helper to help with...waiting
#include "../cpp_lib/pid.cpp"        // waiter helper to help with...waiting
#include "../cpp_lib/wheels.cpp"        // control get raw encoder state

Wheels wheels;
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);
void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void setup() {
  // Open console serial communications
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println("\n\nBeginning initializations...");

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  Serial.println("Setting up interrupts...Done!");

  wheels.updateRadsPerSec(6.283); // 60 RPM
}

long innerDt;
void loop(){
  innerDt = innerWaiter.wait();
  wheels.spin(innerDt);
  if(outerWaiter.isTime()){
    long dt = outerWaiter.starting();
    Serial.print("hi from Outer! Dt: ");
    Serial.println(dt);
  }

}
