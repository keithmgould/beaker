#include "./servoMotor.cpp"

class Wheels {
  private:

  Waiter waiter = Waiter(MOTOR_CONTROL_TIMESTEP);
  Pid leftMotorPid = Pid(MOTOR_CONTROL_TIMESTEP);
  Pid rightMotorPid = Pid(MOTOR_CONTROL_TIMESTEP);

  // bunch of variables for timing...
  float leftPhi, leftLastPhi, leftPhiDot;
  float rightPhi, rightLastPhi, rightPhiDot;
  float leftCommand, rightCommand;

  ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, 1);
  ServoMotor motorRight = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, -1);

  float motorCommand;

  public:

  Wheels(){
    Serial1.begin(9600);
    while (!Serial1) {;}
    leftPhi = leftLastPhi = leftPhiDot = leftCommand = 0;
    rightPhi = rightLastPhi = rightPhiDot = rightCommand = 0;
    leftMotorPid.updateParameters(0.001,0,0); // P, I, D
    rightMotorPid.updateParameters(0.001,0,0); // P, I, D
    leftMotorPid.updateSetpoint(6.283);  // rads/sec rotational velocity of wheels (60rpm)
    rightMotorPid.updateSetpoint(6.283);  // rads/sec rotational velocity of wheels (60rpm)
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

  // in meters. Average of both wheels
  float getX(){
    return (motorLeft.getDistance() + motorRight.getDistance()) / 2.0;
  }

  float getMotorCommand(){
    return motorCommand;
  }

  long getLeftMotorPhi(){
    return motorLeft.getPhi();
  }

  long getRightMotorPhi(){
    return motorRight.getPhi();
  }

  long getLeftMotorEdgeCount(){
    return motorLeft.getEdgeCount();
  }

  long getRightMotorEdgeCount(){
    return motorRight.getEdgeCount();
  }

  // calculates and stores phi and phiDot
  // rads and rads/sec
  void updateLeftPhi(float dt){
    leftPhi = motorLeft.getPhi();
    leftPhiDot = (1000.0 / dt) * (leftPhi - leftLastPhi) / (dt / MOTOR_CONTROL_TIMESTEP);
    leftLastPhi = leftPhi;
  }

  void updateRightPhi(float dt){
    rightPhi = motorRight.getPhi();
    rightPhiDot = (1000.0 / dt) * (rightPhi - rightLastPhi) / (dt / MOTOR_CONTROL_TIMESTEP);
    rightLastPhi = rightPhi;
  }

  void updatePhi(float dt){
    updateRightPhi(dt);
    updateLeftPhi(dt);
  }

  void spin() {
    int dt = waiter.wait(); // will be around 5ms
    updatePhi(dt);

    float leftCommandDelta = leftMotorPid.generateCommand(leftPhiDot, dt);
    float rightCommandDelta = rightMotorPid.generateCommand(rightPhiDot, dt);

    leftCommand += leftCommandDelta;
    rightCommand += rightCommandDelta;

    motorLeft.updatePower(leftCommand);
    motorRight.updatePower(rightCommand);


    String foo = String(leftCommand) + "," + String(rightCommand);
    foo += "," + String(leftPhiDot) + "," + String(rightPhiDot);
    Serial.println(foo);
  }

  void updateRadsPerSec(float rps){
    leftMotorPid.updateSetpoint(rps);
    rightMotorPid.updateSetpoint(rps);
  }
};
