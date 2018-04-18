#include <Math.h>

// TODO Move these to constants file?
#define WHEEL_RADIUS .042 // in meters
#define FULL_ROTATION_EDGE_EVENTS 600 // 18.75 * 32
#define RADS_PER_SEC_TO_RPM 9.5492965855137
#define CLICKS_TO_RADIANS 2 * PI / FULL_ROTATION_EDGE_EVENTS

class ServoMotor
{
  private:

  int edgeCount;
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

  ServoMotor(int firstEncoderPinArg, int secondEncoderPinArg, int tickDirectionArg)
  {
    firstEncoderPin = firstEncoderPinArg;
    secondEncoderPin = secondEncoderPinArg;
    tickDirection = tickDirectionArg;
  }

  void init()
  {

    pinMode(firstEncoderPin, INPUT_PULLUP); // interupt
    pinMode(secondEncoderPin, INPUT_PULLUP); // non-interupt
    edgeCount = 0;
  }

  /*
  Because Sabertooth controls two motors with one 8 byte character,
  when operating in Simplified Serial mode, each motor has 7 bits of resolution.
  Sending a character between 1 and 127 will control motor 1. 1 is full reverse,
  64 is stop and 127 is full forward. Sending a character between 128 and 255
  will control motor 2. 128 is full reverse, 192 is stop and 255 is full forward.
  Character 0 (hex 0x00) is a special case.
  Sending this character will shut down both motors.

  We have seven bits (0-127) so:
  range between -63 and 63.
  -63 is full reverse
  0 is stop
  63 is full forward

  input is a float between -1 and 1
  */
  void updatePower(float raw_power) {
    raw_power = constrain(raw_power, -1, 1) * 63;
    int power = roundf(raw_power);
    byte command = tickDirection > 0 ? 64 + power : 192 + power;
    Serial1.write(command);
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

  int getEdgeCount() {
    return edgeCount;
  }

  void encoderEvent() {
    if(digitalRead(firstEncoderPin) == digitalRead(secondEncoderPin)){
      tickLeft();
    } else {
      tickRight();
    }
  }
};
