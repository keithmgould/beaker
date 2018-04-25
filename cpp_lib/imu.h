#ifndef __BEAKER_IMU__
#define __BEAKER_IMU__

#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU

class Imu{
  private:

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);
  float thetaOffset, theta, thetaDot, lastTheta;

  // in radians
  float rawTheta() {
    imu::Vector<3> eulerDegrees = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    return 0.0174533 * eulerDegrees.z();
  }

  public:

  Imu(){
    thetaOffset = THETA_OFFSET;
    theta = thetaDot = lastTheta = 0;
  }

  void setup(){
  //Check to see if the Inertial Sensor is wired correctly and functioning normally
    Serial.print("Initializing IMU...");
    if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) {
      Serial.println("Inertial Sensor failed, or not present");
      while(true) {;}
    }
    bno.setExtCrystalUse(true);
    Serial.println("Done!");
  }

  float getTheta(){ return theta; }
  float getThetaDot(){ return thetaDot; }

  // calculates and stores theta and thetaDot.
  //
  // dt is assumed to be loop unit. so values should be around 1:
  // dt = actual_loop_time / expected_loop_time.
  //
  // rads and rads/sec
  void update(float dt){
    theta = rawTheta() + thetaOffset;
    thetaDot = (theta - lastTheta) / dt;
    lastTheta = theta;
   }
};

#endif
