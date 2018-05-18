#ifndef __BEAKER_PID__
#define __BEAKER_PID__

#include "Arduino.h"

/*
    This is the classic PID control algorithm. This is probably 
    the most common control algorithm on the planet so not much to add
    here in these comments about how it works.
*/

class Pid {
  private:

  float kP = 0;       // holds the P parameter
  float kI = 0;       // holds the I parameter
  float kD = 0;       // holds the D parameter

  int dt = 0;
  long timestep = 0;
  float currentError = 0;
  float previousError = 0;
  float accumulatedError = 0;
  float deltaError = 0;
  float setpoint = 0;

  // I like self explanatory variable names :)
  bool isClearAccumulatorWhenCrossingZero = false;

  // presentation strings
  String paramString, termString;

    // currentError used for P component
  void setCurrentError(float newError){
    previousError = currentError;
    currentError = newError;
  }

  bool iscrossedZero(){
    return (previousError >= 0 && currentError <= 0) || (previousError < 0 && currentError > 0);
  }

  // // accumulatedError used for I component
  void calculateAccumulatedError(){
    accumulatedError += currentError;

    if(isClearAccumulatorWhenCrossingZero){
      if(iscrossedZero()){
        accumulatedError = 0;
      }
    }

    // helps with windup (overaccumulating the error)
    accumulatedError = constrain(accumulatedError, -1, 1);
  }

  // deltaError is used for the D component of the controller.
  // for that we need the derivative of the error.
  // since this is only called once per loop,
  // we do not need to divide by time.
  void calculateDeltaError(){
    deltaError =  currentError - previousError;
  }

  void buildParamString() {
    paramString = String(kP,4);
    paramString += "," + String(kI,4);
    paramString += "," + String(kD,4);
  }

  public:

  Pid(long ts){
    timestep = ts;
    buildParamString();
  }

  void setClearAccumulatorWhenCrossingZero(bool x){
    isClearAccumulatorWhenCrossingZero = x;
  }

  float getKp(){ return kP; }
  float getKi(){ return kI; }
  float getKd(){ return kD; }
  float getSetpoint(){ return setpoint; }
  float getCurrentError(){ return currentError; }
  String getParamString() { return paramString; }
  String getTermString() { return termString; }

  float pTerm(){ 
    float results = kP * currentError;
    termString = String(results,4);
    return  results;
  }

  float iTerm() { 
    float results = kI * accumulatedError;
    termString += "," + String(results,4);
    return results;
  }

  float dTerm() { 
    float results = kD * deltaError;
    termString += "," + String(results,4);
    return kD * deltaError; 
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
    buildParamString();
  }

  void updateSetpoint(float newSetpoint) {
    setpoint = newSetpoint;
  }

  float generateCommand(float currentState, long newDt){
    updateErrors(currentState, newDt);
    return pTerm() + iTerm() + dTerm();
  }

  // stores parameters at given startingAddress, 
  // and returns the next available address space
  unsigned int storeParameters(unsigned int startingAddress){
    int addr = startingAddress;

    EEPROM.put(addr, kP);
    addr += sizeof(float);
    EEPROM.put(addr, kI);
    addr += sizeof(float);
    EEPROM.put(addr, kD);
    addr += sizeof(float);

    return addr;
  }

  // loads parameters from EEPROM at given startingAddress,
  // and returns the next address space
  unsigned int loadParameters(unsigned int startingAddress){
    float kp, ki, kd;
    unsigned int addr = startingAddress;
    EEPROM.get(addr, kp);
    addr += sizeof(float);
    EEPROM.get(addr, ki);
    addr += sizeof(float);
    EEPROM.get(addr, kd);
    addr += sizeof(float);

    updateParameters(kp, ki, kd);

    return addr;
  }

};

#endif
