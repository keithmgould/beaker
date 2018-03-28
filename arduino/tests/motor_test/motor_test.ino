/*
  This script tests out motors and motor encoders. It's job is just to
  prove out wiring and very basic functionality.

  Expected behavior is to turn both wheels, and spit out x position
  over serial at 115200 BAUD
*/

#include <StandardCplusplus.h>
#include <Wire.h>

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#include "../../constants.cpp"
#include "../../servoMotor.cpp"

// Time (in millisecs) between loops.
// 20 => 50hz
// 4  => 250hz
#define TIMESTEP 20

ServoMotor motorLeft(LH_ENCODER_A,LH_ENCODER_B, 1);
ServoMotor motorRight(RH_ENCODER_A,RH_ENCODER_B, -1);
SoftwareSerial SWSerial(NOT_A_PIN, SerialTX); // RX on no pin (unused). Tx to S1.
SabertoothSimplified sabertooth(SWSerial); // Use SWSerial as the serial port.

long timeMarker = 0;

void leftEncoderEvent() {
  motorLeft.encoderEvent();
}

void rightEncoderEvent() {
  motorRight.encoderEvent();
}

void updatePower(float newGain){
  Serial.print("updating power to: ");
  Serial.println(newGain);
  sabertooth.motor(1, newGain);
  sabertooth.motor(2, newGain);
}

void turnIndicatorLightOff(){
  digitalWrite(INDICATOR, LOW);
}

void turnIndicatorLightOn(){
  digitalWrite(INDICATOR, HIGH);
}

// in meters
float rawLeftXPos() {
  return (motorLeft.getDistance() + motorLeft.getDistance()) / 2.0;
}

// in meters
float rawRightXPos() {
  return (motorRight.getDistance() + motorRight.getDistance()) / 2.0;
}

void setup() {
  pinMode(INDICATOR, OUTPUT);
  // turn off indicator light while we setup
  turnIndicatorLightOff();

  // Open serial communications
  // and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {;}

  // initialize motor encoders and interrupts
  motorLeft.init();
  motorRight.init();

  delay(100);
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, RISING);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, RISING);


  // Sabertooth motor driver commanded over Serial
  SWSerial.begin(9600);
  delay(500);
  while (!SWSerial) {;}
  delay(500);
  updatePower(0);

  // Turn on indicator light because we are ready to rock.
  turnIndicatorLightOn();

  // now that light is on, allow human to get the robot upright
  delay(1000);
  updatePower(30);
  delay(2000);
  updatePower(0);
  delay(500);
  updatePower(-30);
  delay(2000);
  updatePower(0);
}

float av = 0;
float lxpos = 0;
float rxpos = 0;

void loop() {
  long timeDelta = millis() - timeMarker;
  if(timeDelta < TIMESTEP){return;}
  timeMarker = millis();

  lxpos = rawLeftXPos();
  rxpos = rawRightXPos();
  Serial.print(lxpos, 3);
  Serial.print(", ");
  Serial.println(rxpos, 3);
}
