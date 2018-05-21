#include "Arduino.h"
#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C for IMU


/*
		This checks the IMU.

		Check: 

		0. Make sure IMU initializes. Look for: "Done!"
		1. Make sure IMU outputs data. Should change relative to Beaker's theta (angle)
*/


Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);
imu::Vector<3> eulerDegrees;


void setup(){
	Serial.begin(115200); while (!Serial) {;}
  Serial.print("Initializing IMU...");
  if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) {
    Serial.println("Inertial Sensor failed, or not present");
    while(true){;}
  }
  bno.setExtCrystalUse(true);
  Serial.println("Done!");
}

void loop(){
	eulerDegrees = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(eulerDegrees.x());
  Serial.print(",");
  Serial.print(eulerDegrees.y());
  Serial.print(",");
  Serial.println(eulerDegrees.z());
	delay(500);
}