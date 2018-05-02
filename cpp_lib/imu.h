#ifndef __BEAKER_IMU__
#define __BEAKER_IMU__

#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU

class Imu{
  private:

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);
  float thetaOffset, theta, thetaDot;

  // in radians
  float rawTheta() {
    imu::Vector<3> eulerDegrees = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    return 0.0174533 * eulerDegrees.z();
  }

  // in radians/sec
  float rawThetaDot() {
    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    return gyr.z();
  }

  public:

  Imu(){
    thetaOffset = THETA_OFFSET;
    theta = thetaDot = 0;
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
  float getThetaOffset(){ return thetaOffset; }
  void setThetaOffset(float newOffset){ thetaOffset = newOffset; }

  // calculates and stores theta and thetaDot.
  void update(){
    theta = rawTheta() + thetaOffset;
    thetaDot = rawThetaDot();
   }
};

#endif
