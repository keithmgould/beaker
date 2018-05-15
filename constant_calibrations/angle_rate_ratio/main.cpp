#include <Servo.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream>                      // stringstream
#include "../../cpp_lib/includes.h"     // common Beaker functionality


/*
   This Tuning Check lets you try to tune ANGLE_RATE_RATIO.

   See https://github.com/pololu/balboa-32u4-arduino-library/blob/master/examples/Balancer/Balancer.ino

   The below is slightly modified from the above link.

   When the robot is rising toward vertical (not falling),
   angleRate and angle have opposite signs, so this variable
   will just be positive or negative depending on which side of
   vertical it is on.
  
   When the robot is falling, the variable measures how far off
   it is from a trajectory starting it almost perfectly
   balanced then falling to one side or the other with the
   motors off.
  
   Since this depends on ANGLE_RATE_RATIO, it is useful for
   calibration.  If you have changed the wheels or added weight
   to your robot, you can try checking these items, with the
   motor power OFF (powered by USB):
  
   1. Try letting the robot fall "Backwards".
      fallingAngleOffset should be less than 0.  If it
      sometimes positive, reduce ANGLE_RATE_RATIO.
  
   2. If it is tilted "backwards" and given a push back to
      forward again, the fallingAngleOffset should remain positive
      until Beaker hits the ground (ouch!).  
      If fallingAngleOffset is negative, increase ANGLE_RATE_RATIO.
  
   In practice, it is hard to achieve both 1 and 2 perfectly,
   but if you can get close, your constant will probably be
   good enough for balancing.


   Note: data is going out through Bluetooth (Serial3) so that 
   Beaker is less tethered during these calibrations.
*/

Imu my_imu;
PiTalk piTalk;
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);
imu::Vector<3> eulerDegrees;

float angleRateRatioConstant;

void printStuff(int outerDt, float theta, float thetaDot, float fallingAngleOffset){
  String log = String(outerDt);
  log += ", " + String(theta,4);
  log += ", " + String(thetaDot,4);
  log += ", " + String(fallingAngleOffset,4);
  log += ", " + String(angleRateRatioConstant,4);

  Serial3.println(log);
}

void updateAngleRateRatioConstant(std::string message){
  angleRateRatioConstant = String(message.c_str()).toFloat();

  piTalk.sendToPi("Updated ANGLE_RATE_RATIO Constant.");
}

void showHelp(){
  String response = "A: update ANGLE_RATE_RATIO constant.\n";
  response += "H: this help\n";

  piTalk.sendToPi(response);
}

void handlePiTalk(char command, std::string message){
  switch(command){
    case 'A': updateAngleRateRatioConstant(message); break;
    case 'H': showHelp(); break;
  }
}

void setup(){
  piTalk.setup(&my_imu, &handlePiTalk);
  angleRateRatioConstant = 0;

  Serial.begin(115200); while (!Serial) {;}
  Serial3.begin(115200); while (!Serial3) {;}
  Serial.print("Initializing IMU...");
  if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) {
    Serial.println("Inertial Sensor failed, or not present");
    while(true){;}
  }
  bno.setExtCrystalUse(true);
  Serial.println("Done!");
  Outputs::init();
}

void loop(){
  // inner loop behavior
  if(innerWaiter.isTime()){
    innerWaiter.starting();
    my_imu.pushThetaDotData();
  }

  // outer loop behavior
  if(outerWaiter.isTime()){
    float outerDt = outerWaiter.starting();
    my_imu.update();

    float theta = my_imu.getTheta();
    float thetaDot = my_imu.getThetaDot();

    float fallingAngleOffset = thetaDot * angleRateRatioConstant - theta;

    printStuff(outerDt, theta, thetaDot, fallingAngleOffset);

    piTalk.checkForPiCommand();
  }
}