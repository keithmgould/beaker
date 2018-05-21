#ifndef __BEAKER_WHEELS__
#define __BEAKER_WHEELS__

#include "./serialMotor.h"     // low level serial motor control
#include "./waiter.h"
#include "./pid.h"            // Basic PID algorithm

/*
    The Wheels object abstracts away the business of wheels maintaining requested speeds.
    The API is very simple. You tell the Wheels object the desired rads/sec, and the 
    Wheels object will ensure that each wheel exhibits that rads/sec, via the PIDs.

    Each wheel has its own motor. Each motor has a quadrature encoder.
    Each wheel's rotational velocity is controlled by its own PID controller.

    The main loop of the application is expected to have a fast inner loop, around
    5ms, which calls wheels.spin() ever loop. Each of these calls allows the Wheels
    object to adjust the command given to the motor control based on the PID output.
*/

class Wheels {
  private:

  Waiter waiter = Waiter(MOTOR_CONTROL_TIMESTEP);
  Pid leftMotorPid = Pid(MOTOR_CONTROL_TIMESTEP);
  Pid rightMotorPid = Pid(MOTOR_CONTROL_TIMESTEP);
  Averager phiDotAverager = Averager(5);

  // phi is wheel position. In rads.
  float leftPhi, leftLastPhi, rightPhi, rightLastPhi;

  // phiDelta is difference between previous phi and current phi. In rads.
  float leftPhiDelta, rightPhiDelta;

  // phiDot is wheel velocity. in rads/sec.
  float leftPhiDot, rightPhiDot;

  // totalEdgeCount used for calculating phiDelta
  long leftTotalEdgeCount, leftLastTotalEdgeCount, rightTotalEdgeCount, rightLastTotalEdgeCount;

  // result of motor PIDs
  float leftCommandDelta, rightCommandDelta;

  // rads/sec
  float targetRadsPerSec;  

  // actual commands sent to motors
  float leftCommand, rightCommand;

  SerialMotor motorLeft  = SerialMotor(LH_ENCODER_A, LH_ENCODER_B, LEFT);
  SerialMotor motorRight = SerialMotor(RH_ENCODER_A, RH_ENCODER_B, RIGHT);

  // In rads.
  float phiDelta(long totalEdgeCount, long lastTotalEdgeCount){
    // casting safe because edgeDelta <= FULL_ROTATION_EDGE_EVENTS
    float edgeDelta = totalEdgeCount - lastTotalEdgeCount;
    return edgeDelta * CLICKS_TO_RADIANS;
  }

  // calculates and stores LEFT phi and phiDot
  void updateLeftWheelState(float dt){
    leftTotalEdgeCount = motorLeft.getTotalEdgeCount();
    leftPhi = motorLeft.getPhi();
    leftPhiDelta = phiDelta(leftTotalEdgeCount, leftLastTotalEdgeCount);
    leftPhiDot = (1000.0 / dt) * leftPhiDelta / (dt / MOTOR_CONTROL_TIMESTEP);
    leftLastPhi = leftPhi;
    leftLastTotalEdgeCount = leftTotalEdgeCount;
  }

  // calculates and stores RIGHT phi and phiDot
  void updateRightWheelState(float dt){
    rightTotalEdgeCount = motorRight.getTotalEdgeCount();
    rightPhi = motorRight.getPhi();
    rightPhiDelta = phiDelta(rightTotalEdgeCount, rightLastTotalEdgeCount);
    rightPhiDot = (1000.0 / dt) * rightPhiDelta / (dt / MOTOR_CONTROL_TIMESTEP);
    rightLastPhi = rightPhi;
    rightLastTotalEdgeCount = rightTotalEdgeCount;
  }

  void updateWheelStates(float dt){
    updateRightWheelState(dt);
    updateLeftWheelState(dt);
  }

  public:

  Wheels(){
    // Serial1 used for communication with motor driver (Sabertooth)
    Serial1.begin(9600); while (!Serial1) { delay(10); }
  }

  void initialize(){
    motorLeft.updatePower(0);
    motorRight.updatePower(0);
    delay(500);
    updateRadsPerSec(0);
    updatePids(MOTOR_P_PARAM,MOTOR_I_PARAM,MOTOR_D_PARAM);
    resetCounts();
  }

  // called by interrupt callback
  void rightEncoderEvent(){
    motorRight.encoderEvent();
  }

  // called by interrupt callback  
  void leftEncoderEvent(){
    motorLeft.encoderEvent();
  }

  // this method "trusts" that the robot is not moving when 
  // this method is called. Otherwise, Bad Things might happen.
  void resetCounts(){
    targetRadsPerSec = 0;
    leftPhi = leftLastPhi = rightPhi = rightLastPhi = 0;
    leftPhiDelta = rightPhiDelta = 0;
    leftPhiDot = rightPhiDot = 0;
    leftCommandDelta = rightCommandDelta = 0;
    leftTotalEdgeCount = leftLastTotalEdgeCount = rightTotalEdgeCount = rightLastTotalEdgeCount = 0;
    motorLeft.resetCount();
    motorRight.resetCount();
  }

  // in meters. Average of both wheels
  float getX(){
    return (motorLeft.getDistance() + motorRight.getDistance()) / 2.0;
  }

  float getTargetRadsPerSec() { return targetRadsPerSec; }              // rads/sec
  float getPhi(){ return (leftPhi + rightPhi) / 2.0; }                  // rads. avg of 2 whls
  float getPhiDot(){ return (leftPhiDot + rightPhiDot) / 2.0; }         // rads/sec. avg of 2 whls
  float getPhiDotAvg(){ return phiDotAverager.computeAverage(); }       // rads/sec. avg of 2 whls
  float getLeftPhi(){ return leftPhi; }                                 // rads
  float getLeftLastPhi(){ return leftLastPhi; }                         // rads
  float getLeftCommand(){ return leftCommand; }                         // power. ranges from -1 to 1
  float getLeftPhiDelta(){ return leftPhiDelta; }                       // rads
  float getRightPhiDelta(){ return rightPhiDelta; }                     // rads
  float getLeftPhiDot(){ return leftPhiDot; }                           // rads/sec
  float getRightPhi(){ return rightPhi; }                               // rads
  float getRightPhiDot(){ return rightPhiDot; }                         // rads/sec
  float getLeftSetpoint(){ return leftMotorPid.getSetpoint(); }         // rads/sec
  float getRightSetpoint(){ return rightMotorPid.getSetpoint(); }       // rads/sec
  String getLeftPidParams(){ return leftMotorPid.getParamString(); }    // Kp, Ki, Kd values
  String getRightPidParams(){ return rightMotorPid.getParamString(); }  // Kp, Ki, Kd values
  String getLeftPidTerms(){ return leftMotorPid.getTermString(); }  // Kp, Ki, Kd values

  // ths method is called every inner (fast) loop, usually 
  // around every 5ms.
  //
  // its job is to keep the wheels spinning at the desired rad/sec.
  //
  // it fetches the wheel states, then uses each motor's PID controller
  // to calculate a new command. Finally it issues that command.
  void spin(long dt) {
    updateWheelStates(dt);
    phiDotAverager.push(getPhiDot());
    leftCommandDelta = leftMotorPid.generateCommand(leftPhiDot, dt);
    rightCommandDelta = rightMotorPid.generateCommand(rightPhiDot, dt);
    leftCommand += leftCommandDelta;
    rightCommand += rightCommandDelta;
    motorLeft.updatePower(leftCommand);
    motorRight.updatePower(rightCommand);
  }

  // For tuning the motor PIDs. This should not
  // be needed, as default values in the constants.h
  // are pretty good, but it's here if needed.
  void updatePids(float kp, float ki, float kd){
    leftMotorPid.updateParameters(kp, ki, kd);
    rightMotorPid.updateParameters(kp, ki, kd);
  }

  // This is the main method used to control the wheels.
  // Send the required rads/sec, and Wheels will make sure
  // that happens via PID
  void updateRadsPerSec(float rps){
    targetRadsPerSec = rps;
    leftMotorPid.updateSetpoint(rps);
    rightMotorPid.updateSetpoint(rps);
  }
};

#endif
