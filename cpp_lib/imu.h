#ifndef __BEAKER_IMU__
#define __BEAKER_IMU__

#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU
#include <vector>

/*
    The Inertial Measurement Unit (IMU) reads from (currently) the BNO055.
    We might want to abstract away the specific chip at a later date. For now
    this class is simply a convenience wrapper for the BNO055 IMU.

    The readouts involve theta, Beakers angle from "balanced," and 
    thetaDot, which is the rate of change of theta.

    Theta is produced via black-box IMU magic, and thetaDot is produced
    via the IMU's gyrometer.
*/
class Imu{
  private:

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);
  float thetaOffset, theta, thetaDot;
  std::vector<float> thetaDotData;

  // in radians
  float rawTheta() {
    imu::Vector<3> eulerDegrees = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    return 0.0174533 * eulerDegrees.z();
  }

  // in radians/sec
  float rawThetaDot() {
    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    return -gyr.x();
  }

  float computeThetaDotAvg(){
    float avg = 0;
    
    std::for_each(thetaDotData.begin(), thetaDotData.end(), [&] (float n) {
      avg += n;
    });

    avg = avg / (float) thetaDotData.size();

    thetaDotData.clear();

    return avg;
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
      Outputs::permaError();
    }
    bno.setExtCrystalUse(true);
    Serial.println("Done!");
  }

  void pushThetaDotData(){
    thetaDotData.push_back(rawThetaDot());
  }

  float getTheta(){ return theta; }
  float getThetaDot(){ return thetaDot; }
  float getThetaOffset(){ return thetaOffset; }

  // Due to micro leveling imperfections, the IMU is not perfectly level when Beaker
  // is balanced. This offset allows software to account for this and update when
  // necessary.
  void setThetaOffset(float newOffset){ thetaOffset = newOffset; }

  // calculates and stores theta and thetaDot.
  void update(){
    theta = rawTheta() + thetaOffset;
    thetaDot = computeThetaDotAvg();
   }
};

#endif
