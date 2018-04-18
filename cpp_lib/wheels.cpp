#include "./servoMotor.cpp"

class Wheels {
  private:

  ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, 1);
  ServoMotor motorRight = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, -1);
  float previousGain;

  public:

  Wheels(){
    Serial1.begin(9600);
    while (!Serial1) {;}
  }

  void rightEncoderEvent(){
    motorRight.encoderEvent();
  }

  void leftEncoderEvent(){
    motorLeft.encoderEvent();
  }

  // in radians. Average of both wheels
  float getPhi(){
    return (motorLeft.getPhi() + motorRight.getPhi()) / 2.0;
  }

  float getX(){
    return (motorLeft.getDistance() + motorRight.getDistance()) / 2.0;
  }

  void command(float motorCommand){
    // safety first. ensure gain between these values
    float newGain = constrain(motorCommand, -1, 1);
    if(newGain == previousGain){ return; }
    previousGain = newGain;
    motorLeft.updatePower(newGain);
    motorRight.updatePower(newGain);
  }
};
