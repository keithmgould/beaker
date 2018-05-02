#ifndef __BEAKER_MOTOR__
#define __BEAKER_MOTOR__
#include "Arduino.h"
#include <Math.h>

// TODO Move these to constants file?
#define WHEEL_RADIUS .042 // in meters
#define FULL_ROTATION_EDGE_EVENTS 600 // 18.75 * 32
#define RADS_PER_SEC_TO_RPM 9.5492965855137
#define CLICKS_TO_RADIANS 2 * PI / FULL_ROTATION_EDGE_EVENTS

class Motor
{
  private:

  long edgeCount;
  int firstEncoderPin, secondEncoderPin, tickDirection;

  void tickRight()
  {
    edgeCount -= tickDirection;
  }

  void tickLeft()
  {
    edgeCount += tickDirection;
  }

  //-------------------------------------------------------------------
  public:
  //-------------------------------------------------------------------

  Motor(int firstEncoderPinArg, int secondEncoderPinArg, int tickDirectionArg)
  {
    firstEncoderPin = firstEncoderPinArg;
    secondEncoderPin = secondEncoderPinArg;
    tickDirection = tickDirectionArg;

    pinMode(firstEncoderPin, INPUT_PULLUP); // interupt
    pinMode(secondEncoderPin, INPUT_PULLUP); // non-interupt
    edgeCount = 0;
  }

  void resetCount() {
    edgeCount = 0;
  }

  // in radians
  float getPhi() {
    return (float) edgeCount * (float) CLICKS_TO_RADIANS;
  }

  // in meters
  float getDistance() {
    return getPhi() * (float) WHEEL_RADIUS;
  }

  long getEdgeCount() {
    return edgeCount;
  }

  // called by main encoder interrupt
  void encoderEvent() {
    if(digitalRead(firstEncoderPin) == digitalRead(secondEncoderPin)){
      tickLeft();
    } else {
      tickRight();
    }
  }
};

#endif
