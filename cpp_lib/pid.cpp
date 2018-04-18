#include "Arduino.h"

class Pid {
  private:


  float kP = 0;       // holds the P parameter
  float kI = 0;       // holds the I parameter
  float kD = 0;       // holds the D parameter

  int dt;
  long timestep;
  float currentError;
  float previousError;
  float accumulatedError;
  float deltaError;
  float setpoint;

    // currentError used for P component
  void setCurrentError(float newError){
    previousError = currentError;
    currentError = newError;
  }

  // // accumulatedError used for P component
  void calculateAccumulatedError(){
    // once we cross the 0 error mark,
    // the past accumulated error
    // is no longer helping us. Reset to 0.
    if(currentError == 0){
      accumulatedError = 0;
      return;
    }

    accumulatedError += currentError;

    // helps with windup (overaccumulating the error)
    accumulatedError = constrain(accumulatedError, -1, 1);
  }

  // deltaError is used for the D component of the controller.
  // for that we need the derivative of the error.
  // since this is only called once per loop,
  // we do not need to divide by time.
  // deltaError used for D component
  void calculateDeltaError(){
    deltaError =  currentError - previousError;
  }

  public:

  Pid(long ts){
    timestep = ts;
  }

  void updateErrors(float currentState, long newDt){
    float newError = setpoint - currentState;
    dt = newDt;
    setCurrentError(newError);
    calculateAccumulatedError();
    calculateDeltaError();
  }

  void updateParameters(float p, float i, float d) {
    kP = p;
    kI = i;
    kD = d;
  }

  void updateSetpoint(float newSetpoint) {
    setpoint = newSetpoint;
  }

  float generateCommand(){
    return kP * currentError + kI * accumulatedError + kD * deltaError;
  }

  float generateCommand(float currentState, long newDt){
    updateErrors(currentState, newDt);
    return kP * currentError + kI * accumulatedError + kD * deltaError;
  }

};

// end of PID class------------------------------------
