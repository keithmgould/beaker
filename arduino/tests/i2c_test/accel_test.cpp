#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "../../constants.cpp"

// IMU constants
#define BALANCED_OFFSET 0.89 // sensor does not show 0 on balance but it should.

// pi / 180, for degrees to radians
#define PI_OVER_ONE_EIGHTY 0.017453292519943

#define TIMESTEP 20 // out of 1000 (ms)

float currentTheta = 0; // can we do float() to declare instead of 0?

Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);

float degToRadians(float deg) {
  return deg * PI_OVER_ONE_EIGHTY;
}

// "acceleration" to radians
// sin(theta) = y / 9.8.
// solve for theta
float accToRadians(float acc) {
  float theta = (acc + BALANCED_OFFSET) / 9.8;
  if(theta < -1) { theta = -1; }
  if(theta > 1) { theta = 1; }
  return asin(theta);
}

float degToRadians(float deg) {
  return deg * PI_OVER_ONE_EIGHTY;
}

// radians per second
float rawThetaDot(){
  // Angular velocity in degrees per second (needs to be converted)
  imu::Vector<3> RotationalVelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  return degToRadians(RotationalVelocity.y());
}

// in radians
float rawTheta() {
  // Linear Acceleration in meters per second squared
  imu::Vector<3> Acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  return accToRadians(-Acceleration.z());
}

// radians per second
float rawBodyAngularVelocity(){
  // Angular velocity in degrees per second (needs to be converted)
  imu::Vector<3> RotationalVelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  return degToRadians(RotationalVelocity.y());
}

void reportError() {
  pinMode(INDICATOR, OUTPUT);
  while(true){
    digitalWrite(INDICATOR, HIGH);
    delay(300);
    digitalWrite(INDICATOR, LOW);
    delay(300);
  }
}

void setup() {
  pinMode(INDICATOR, OUTPUT);
  // turn off indicator light while we setup
  digitalWrite(INDICATOR, LOW);

  // Open serial communications
  // and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {}

  //Check to see if the Inertial Sensor is wired correctly and functioning normally
  if (!bno.begin(0x05)) {
    Serial.println("Inertial Sensor failed, or not present");
    reportError();
  } else {

    bno.setExtCrystalUse(true);

    Serial.println("Inertial Sensor present");
  }

  // Turn on indicator light because we are ready to rock.
  digitalWrite(INDICATOR, HIGH);
}

void loop() {
  // theta in radians
  Serial.print(rawTheta(), 5);
  Serial.print(",");

  // theta dot in radians (raw)
  Serial.println(rawBodyAngularVelocity(),5);

  delay(TIMESTEP);
}









