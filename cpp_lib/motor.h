#ifndef __BEAKER_MOTOR__
#define __BEAKER_MOTOR__
#include "Arduino.h"
#include <Math.h>

// TODO Move these to constant.h?
#define WHEEL_RADIUS .042 // in meters
#define WHEEL_CIRCUMFERECE 2 * PI * WHEEL_RADIUS
#define FULL_ROTATION_EDGE_EVENTS 600 // 18.75 * 32
#define CLICKS_TO_RADIANS 2 * PI / FULL_ROTATION_EDGE_EVENTS

/*
    The motor object encapsulates the encoders which means
    it is in charge of phi (wheel position) and x (linear distance).

    Subclasses of motor (servoMotor and serialMotor) encapsulate 
    telling the motor how much voltage to apply.

    The motor class, and its subclasses, are not aware of time, so they 
    do not keep track of speeds, either rotational or linear.
*/

class Motor
{
  private:

  int edgeCount; // holds the edge counts within a single rotation. always between 0 and FULL_ROTATION_EDGE_EVENTS
  long rotations; // keeps track of rotations. Needed for X position.
  int tickDirection; // keeps track of increment/decrement when the interrupt from encoder hits.
  int firstEncoderPin, secondEncoderPin;

  void handleInfinity(){
    if (edgeCount < 0){
      rotations--;
      edgeCount = FULL_ROTATION_EDGE_EVENTS;
    }
    if (edgeCount > FULL_ROTATION_EDGE_EVENTS){
      rotations++;
      edgeCount = 0;
    }
  }

  void tickRight()
  {
    edgeCount -= tickDirection;
    handleInfinity();
  }

  void tickLeft()
  {
    edgeCount += tickDirection;
    handleInfinity();
  }

  //-------------------------------------------------------------------
  public:
  //-------------------------------------------------------------------

  void resetCount() {
    edgeCount = rotations = 0;
  }

  Motor(int firstEncoderPinArg, int secondEncoderPinArg, int tickDirectionArg)
  {
    firstEncoderPin = firstEncoderPinArg;
    secondEncoderPin = secondEncoderPinArg;
    tickDirection = tickDirectionArg;

    pinMode(firstEncoderPin, INPUT_PULLUP); // interupt
    pinMode(secondEncoderPin, INPUT_PULLUP); // non-interupt
    resetCount();
  }

  // in radians
  float getPhi() {
    return (float) edgeCount * (float) CLICKS_TO_RADIANS;
  }

  // in meters  
  float rotationsToMeters(){
    return rotations * WHEEL_CIRCUMFERECE;
  }

  // in meters
  float getDistance() {
    return rotationsToMeters() + getPhi() * (float) WHEEL_RADIUS;
  }

  long getEdgeCount() {
    return edgeCount;
  }

  int getDirection(){
    return tickDirection;
  }

  // this should be called by encoder interrupts
  void encoderEvent() {
    if(digitalRead(firstEncoderPin) == digitalRead(secondEncoderPin)){
      tickLeft();
    } else {
      tickRight();
    }
  }
};

#endif
