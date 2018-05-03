#ifndef __BEAKER_WHEELS__
#define __BEAKER_WHEELS__

#include "./servoMotor.h"     // low level motor control
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

  float leftPhi, leftLastPhi, rightPhi, rightLastPhi, leftPhiDot, rightPhiDot;

  // actual commands sent to motors
  float leftCommand, rightCommand;

  ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, FORWARD);
  ServoMotor motorRight = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, BACKWARD);

  // phiDelta is distance in radians between current and prev phi value.
  // given the modulus nature of phi (always has a value betwenm 0 to FULL_ROTATION_EDGE_EVENTS),
  // and direction of rotation, there are four different ways to calculate phiDelta.
  float phiDelta(float phi, float lastPhi, int direction){
    if(phi == lastPhi) { return 0; }

    if(direction == FORWARD){
      if(phi > lastPhi){
        return phi - lastPhi;
      }else{
        return FULL_ROTATION_EDGE_EVENTS - lastPhi + phi;
      }
    }else{
      if(phi > lastPhi){
        return phi - FULL_ROTATION_EDGE_EVENTS - lastPhi;
      }else{
        return 0 - lastPhi + phi;
      }
    }
  }

  // calculates and stores LEFT phi and phiDot
  void updateLeftPhi(float dt){
    leftPhi = motorLeft.getPhi();
    float pDelta = phiDelta(leftPhi, leftLastPhi, motorLeft.getDirection());
    leftPhiDot = (1000.0 / dt) * pDelta / (dt / MOTOR_CONTROL_TIMESTEP);
    String foo = "dt: " + String(dt) + ", leftPhi: " + String(leftPhi) + ", leftLastPhi: " + String(leftLastPhi) + ", phiDelta: " + String(pDelta); 
    // Serial.println(foo);
    leftLastPhi = leftPhi;
  }

  // calculates and stores RIGHT phi and phiDot
  void updateRightPhi(float dt){
    rightPhi = motorRight.getPhi();
    float pDelta = phiDelta(rightPhi, rightLastPhi, motorRight.getDirection());
    rightPhiDot = (1000.0 / dt) * pDelta / (dt / MOTOR_CONTROL_TIMESTEP);
    rightLastPhi = rightPhi;
  }

  void updatePhi(float dt){
    updateRightPhi(dt);
    updateLeftPhi(dt);
  }

  // resets the quad encoder tick count for each motor.
  void resetPhi(){
    motorLeft.resetCount();
    motorRight.resetCount();
  }

  public:

  Wheels(){
    // Serial1 used for communication with motor driver (Sabertooth)
    Serial1.begin(9600);
    while (!Serial1) {;}

    leftPhi = leftLastPhi = leftPhiDot = leftCommand = 0;
    rightPhi = rightLastPhi = rightPhiDot = rightCommand = 0;
  }

  void initialize(){
    motorLeft.attach(LEFT_MOTOR_DRIVER);
    motorRight.attach(RIGHT_MOTOR_DRIVER);
    updateRadsPerSec(0);
    updatePids(MOTOR_P_PARAM,MOTOR_I_PARAM,MOTOR_D_PARAM);
    resetPhi();
  }

  // called by interrupt callback
  void rightEncoderEvent(){
    motorRight.encoderEvent();
  }

  // called by interrupt callback  
  void leftEncoderEvent(){
    motorLeft.encoderEvent();
  }

  // in meters. Average of both wheels
  float getX(){
    return (motorLeft.getDistance() + motorRight.getDistance()) / 2.0;
  }

  long getPhi(){ return (leftPhi + rightPhi) / 2.0; }                   // rads. avg of 2 whls
  float getPhiDot(){ return (leftPhiDot + rightPhiDot) / 2.0; }         // rads/sec. avg of 2 whls
  long getLeftPhi(){ return leftPhi; }                                  // rads
  float getLeftPhiDot(){ return leftPhiDot; }                           // rads/sec
  long getRightPhi(){ return rightPhi; }                                // rads
  float getRightPhiDot(){ return rightPhiDot; }                         // rads/sec
  long getLeftMotorEdgeCount(){ return motorLeft.getEdgeCount(); }      // encoder ticks
  long getRightMotorEdgeCount(){ return motorRight.getEdgeCount(); }    // encoder ticks

  void spin(long dt) {
    updatePhi(dt);

    float leftCommandDelta = leftMotorPid.generateCommand(leftPhiDot, dt);
    float rightCommandDelta = rightMotorPid.generateCommand(rightPhiDot, dt);

    leftCommand += leftCommandDelta;
    rightCommand += rightCommandDelta;

    String foo = "setPoint: " + String(leftMotorPid.getSetpoint()) + ", lpd: " + String(leftPhiDot) + ", dt: " + String(dt) + ", leftCommandDelta: " + String(leftCommandDelta) + ", leftCommand: " + String(leftCommand); 
    Serial.println(foo);
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
    leftMotorPid.updateSetpoint(rps);
    rightMotorPid.updateSetpoint(rps);
  }
};

#endif
