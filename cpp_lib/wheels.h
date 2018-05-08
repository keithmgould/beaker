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

  // phi is wheel position. In rads.
  float leftPhi, leftLastPhi, rightPhi, rightLastPhi;

  // phiDelta is difference between previous phi and current phi. In rads.
  float leftPhiDelta, rightPhiDelta;

  // phiDot is wheel velocity. in rads/sec.
  float leftPhiDot, rightPhiDot;

  // XPos is distance. In Meters.
  float leftXPos, leftLastXPos, rightXPos, rightLastXPos;

  // result of motor PIDs
  float leftCommandDelta, rightCommandDelta;

  // actual commands sent to motors
  float leftCommand, rightCommand;

  ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, LEFT);
  ServoMotor motorRight = ServoMotor(RH_ENCODER_A, RH_ENCODER_B, RIGHT);

  // phiDelta is distance in radians between current and prev phi value.
  // given the modulus nature of phi (always has a value betwenm 0 to 2*PI),
  // and direction of rotation, there are four different ways to calculate phiDelta.
  // Direction of rotation is inferred from distanfe being positive or negative
  float phiDelta(float phi, float lastPhi, float distance){
    if(phi == lastPhi) { return 0; }

    if(distance > 0){                         // If true, wheel moving forward
      if(phi > lastPhi){                      // if true, phi did not reset
        // Serial.print("A!!!");
        return phi - lastPhi;                 // vanilla case of higher new phi minus lower old phi
      }else{                                  // phi passed 2PI and reset
        // Serial.print("B!!!");
        return TWO_PI - lastPhi + phi;        // calculate delta across the reset point
      }
    }else{                                    // wheel moving backward
      if(phi > lastPhi){                      // phi passed 0/2PI and reset
        // Serial.print("C!!!");
        return phi - TWO_PI - lastPhi;        // calculate delta across reset point
      }else{                                  // vanilla case of subtracting within a rotation
        // Serial.print("D!!!");
        return 0 - lastPhi + phi;             // calculate phiDelta
      }
    }
  }

  // calculates and stores LEFT phi and phiDot
  void updateLeftWheelState(float dt){
    leftXPos = motorLeft.getDistance();
    leftPhi = motorLeft.getPhi();
    // Serial.print("phi: "); Serial.print(leftPhi,5); Serial.print(", lastPhi: "); Serial.print(leftLastPhi,5);
    // Serial.print(", xPos: "); Serial.print(leftXPos,5);Serial.print(", lastxPos: "); Serial.print(leftLastXPos,5);
    leftPhiDelta = phiDelta(leftPhi, leftLastPhi, leftXPos - leftLastXPos);
    leftPhiDot = (1000.0 / dt) * leftPhiDelta / (dt / MOTOR_CONTROL_TIMESTEP);
    leftLastPhi = leftPhi;
    leftLastXPos = leftXPos;
  }

  // calculates and stores RIGHT phi and phiDot
  void updateRightWheelState(float dt){
    rightXPos = motorRight.getDistance();
    rightPhi = motorRight.getPhi();
    rightPhiDelta = phiDelta(rightPhi, rightLastPhi, rightXPos - rightLastXPos);
    rightPhiDot = (1000.0 / dt) * rightPhiDelta / (dt / MOTOR_CONTROL_TIMESTEP);
    rightLastPhi = rightPhi;
    rightLastXPos = rightXPos;
  }

  void updateWheelStates(float dt){
    // updateRightWheelState(dt);
    updateLeftWheelState(dt);
  }

  void resetCounts(){
    leftPhi = leftLastPhi = rightPhi = rightLastPhi = 0;
    leftPhiDelta = rightPhiDelta = 0;
    leftPhiDot = rightPhiDot = 0;
    leftXPos = leftLastXPos = rightXPos = rightLastXPos = 0;
    leftCommandDelta = rightCommandDelta = 0;
    motorLeft.resetCount();
    motorRight.resetCount();
  }

  public:

  Wheels(){
    // Serial1 used for communication with motor driver (Sabertooth)
    Serial1.begin(9600); while (!Serial1) { delay(100); }
  }

  void initialize(){
    motorLeft.attach(LEFT_MOTOR_DRIVER);
    motorRight.attach(RIGHT_MOTOR_DRIVER);
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

  // in meters. Average of both wheels
  float getX(){
    return (motorLeft.getDistance() + motorRight.getDistance()) / 2.0;
  }

  float getPhi(){ return (leftPhi + rightPhi) / 2.0; }                  // rads. avg of 2 whls
  float getPhiDot(){ return (leftPhiDot + rightPhiDot) / 2.0; }         // rads/sec. avg of 2 whls
  float getLeftPhi(){ return leftPhi; }                                 // rads
  float getLeftLastPhi(){ return leftLastPhi; }                         // rads
  float getLeftCommand(){ return leftCommand; }                         // power. ranges from -1 to 1
  float getLeftPhiDelta(){ return leftPhiDelta; }                       // rads
  float getRightPhiDelta(){ return rightPhiDelta; }                     // rads
  float getLeftPhiDot(){ return leftPhiDot; }                           // rads/sec
  float getRightPhi(){ return rightPhi; }                               // rads
  float getRightPhiDot(){ return rightPhiDot; }                         // rads/sec
  float getLeftDistance() { return motorLeft.getDistance(); }           // meters
  float getRightDistance() { return motorRight.getDistance(); }         // meters
  long getLeftMotorEdgeCount(){ return motorLeft.getEdgeCount(); }      // encoder ticks
  long getRightMotorEdgeCount(){ return motorRight.getEdgeCount(); }    // encoder ticks
  float getLeftSetpoint(){ return leftMotorPid.getSetpoint(); }         // rads/sec
  float getRightSetpoint(){ return rightMotorPid.getSetpoint(); }       // rads/sec
  String getLeftPidParams(){ return leftMotorPid.getParamString(); }    // Kp, Ki, Kd values
  String getRightPidParams(){ return rightMotorPid.getParamString(); }  // Kp, Ki, Kd values

  void spin(long dt) {
    updateWheelStates(dt);
    leftCommandDelta = leftMotorPid.generateCommand(leftPhiDot, dt);
    rightCommandDelta = rightMotorPid.generateCommand(rightPhiDot, dt);
    leftCommand += leftCommandDelta;
    rightCommand += rightCommandDelta;

    String foo = ", dt: " + String(dt);
    foo += ", leftPhiDot: " + String(leftPhiDot,5);
    // foo += ", leftPhiDelta: " + String(leftPhiDelta,5);
    // foo += ", leftCommandDelta: " + String(leftCommandDelta,5);
    // foo += ", leftCommand: " + String(leftCommand,5);

    Serial.println(foo); 
    motorLeft.updatePower(leftCommand);
    // motorRight.updatePower(rightCommand);
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
