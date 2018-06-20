#ifndef __BEAKER_MOTOR__
#define __BEAKER_MOTOR__
#include "Arduino.h"
#include <Math.h>

/*
    The motor object encapsulates the encoders which means
    it is in charge of phi (wheel position) and x (linear distance).

    Subclasses of motor (servoMotor and serialMotor) encapsulate 
    telling the motor how much voltage to apply.

    The motor class, and its subclasses, are not aware of time, so they 
    do not keep track of speeds, either rotational or linear.

    Phi is determined using the motor's quadrature encoders.
    One rotation has FULL_ROTATION_EDGE_EVENTS ticks or "edge events."
    The "edge count" is how many ticks registered in current rotation.
*/

class Motor
{
  protected:

  // keeps track of increment/decrement when the interrupt from encoder hits. 
  int tickDirection; 


  private:

  int edgeCount; // holds the edge counts within a single rotation. always between 0 and FULL_ROTATION_EDGE_EVENTS
  long totalEdgeCount; // holds the edge counts across all rotations.
  long rotations; // keeps track of number of rotations. Needed for X position.
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
    edgeCount += tickDirection;
    totalEdgeCount += tickDirection;
    handleInfinity();
  }

  void tickLeft()
  {
    edgeCount -= tickDirection;
    totalEdgeCount -= tickDirection;
    handleInfinity();
  }

  // in meters  
  float rotationsToMeters(){
    return rotations * WHEEL_CIRCUMFERECE;
  }

  //-------------------------------------------------------------------
  public:
  //-------------------------------------------------------------------

  void resetCount() {
    edgeCount = totalEdgeCount = rotations = 0;
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

  // in radians. between 0 and 2PI
  float getPhi() {
    return (float) edgeCount * CLICKS_TO_RADIANS;
  }

  // distance in radians.
  float getTotalPhi() {
      return (float) rotations * TWO_PI + getPhi();
  }

  // in meters
  float getDistance() {
    return rotationsToMeters() + getPhi() * WHEEL_RADIUS;
  }

  long getTotalEdgeCount() {
    return totalEdgeCount;
  }

  // between 0 and FULL_ROTATION_EDGE_EVENTS
  int getEdgeCount() {
    return edgeCount;
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
