#include "./servoMotor.cpp"

class Wheels {
  private:

  Waiter waiter = Waiter(MOTOR_CONTROL_TIMESTEP);
  Pid leftMotorPid = Pid(MOTOR_CONTROL_TIMESTEP);
  Pid rightMotorPid = Pid(MOTOR_CONTROL_TIMESTEP);

  // bunch of variables for timing...
  float leftPhi, leftLastPhi, leftPhiDot;
  float rightPhi, rightLastPhi, rightPhiDot;

  // actual commands sent to motors
  float leftCommand, rightCommand;

  ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, 1);
  ServoMotor motorRight = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, -1);

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

  public:

  Wheels(){
    // Serial1 used for communication with motor driver
    Serial1.begin(9600);
    while (!Serial1) {;}

    leftPhi = leftLastPhi = leftPhiDot = leftCommand = 0;
    rightPhi = rightLastPhi = rightPhiDot = rightCommand = 0;
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

  void updatePhi(float dt){
    updateRightPhi(dt);
    updateLeftPhi(dt);
  }



  // float logs[2][200];
  long counter = 0;
  bool storeLogs = false;

  void spin(long dt) {
    updatePhi(dt);

    float leftCommandDelta = leftMotorPid.generateCommand(leftPhiDot, dt);
    float rightCommandDelta = rightMotorPid.generateCommand(rightPhiDot, dt);

    leftCommand += leftCommandDelta;
    rightCommand += rightCommandDelta;

    motorLeft.updatePower(leftCommand);
    motorRight.updatePower(rightCommand);

    // if(storeLogs){
    //   logs[0][counter] = leftPhiDot;
    //   logs[1][counter] = rightPhiDot;
    //   counter++;
    //   if(counter >= 195){
    //     storeLogs = false;
    //     counter = 0;
    //     Serial.println("Log Time!");
    //     for(int i = 0;i<195;i++){
    //       String foo =  String(i) + "," +String(logs[0][i],5) + "," + String(logs[1][i],5);
    //       Serial.println(foo);
    //     }
    //   }
    // }
  }

  void updatePids(float kp, float ki, float kd){
    leftMotorPid.updateParameters(kp, ki, kd);
    rightMotorPid.updateParameters(kp, ki, kd);
  }

  void updateRadsPerSec(float rps){
    leftMotorPid.updateSetpoint(rps);
    rightMotorPid.updateSetpoint(rps);
    counter = 0;
    storeLogs = true;
  }
};
