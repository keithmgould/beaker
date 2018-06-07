#include "Arduino.h"
#include <Servo.h>
#include "../../lib/constants.h"
#include "../../lib/serialMotor.h"

/*

    NOTE: Make sure Beaker is wired and configured for SERIAL communication to Sabertooth, not PWM.
  
    NOTE: This test is meant to be performed with Beaker hanging, wheels NOT touching the ground.

    In this characteristic check:

    The wheels are brought up to 10 rads per sec, then the power to the motor driver is shut off
    and the time taken for the wheels to stop is measured and reported back. 

    It's possible (likely) that the two wheels have different frictions. Keep track which is which!
    There are "Left" (L) and "Right" (R) indicators on each of Beaker's motors.

    The test will automatically be repeated 10 times, and all values plus an average will be given.
*/

SerialMotor motorLeft  = SerialMotor(LH_ENCODER_A, LH_ENCODER_B, LEFT);
SerialMotor motorRight  = SerialMotor(RH_ENCODER_A, RH_ENCODER_B, RIGHT);
void leftEncoderEvent(){ motorLeft.encoderEvent(); }
void rightEncoderEvent(){ motorRight.encoderEvent(); }



void iteration(){
  motorLeft.updatePower(0.5);
  motorRight.updatePower(0.5);
  delay(5000); // give a few seconds to get up to speed
  long leftFirstTotalEdgeCount = motorLeft.getTotalEdgeCount();
  long rightFirstTotalEdgeCount = motorRight.getTotalEdgeCount();
  delay(1000); // wait a sec for speed measurement
  long leftSecondTotalEdgeCount = motorLeft.getTotalEdgeCount();
  long rightSecondTotalEdgeCount = motorRight.getTotalEdgeCount();
  long startSlowdown = millis();
  byte powerDown = 0;
  Serial1.write(powerDown);

  long leftNewTotalEdgeCount = 0;
  long leftOldTotalEdgeCount = 0;
  long rightNewTotalEdgeCount = 0;
  long rightOldTotalEdgeCount = 0;

  while(true){
    leftNewTotalEdgeCount = motorLeft.getTotalEdgeCount();
    rightNewTotalEdgeCount = motorRight.getTotalEdgeCount();
    if(leftNewTotalEdgeCount == leftOldTotalEdgeCount && rightNewTotalEdgeCount == rightOldTotalEdgeCount){
      Serial.print("Stopped Spinning!");
    }
  }

}

void setup(){
  Serial1.begin(9600); while (!Serial1) {delay(1);}
  Serial.begin(115200); while (!Serial) {delay(1);}
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  iteration();

}

void loop(){}