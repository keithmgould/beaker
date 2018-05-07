/*
  Beaker's Arduino communicates with the Raspberry Pi over Serial.

  Communication is bidirectional.

  This library facilitates.

  When a command comes down from the Pi, PiTalk checks to see if it can 
  handle the command itself. If it can't, it passes the command off
  to the main algorithm via callback.

  Commands take the form <single letter command><details of command>

  **If the details are an array, they are space delimited.**

  Ex: 

  M0.15 0.001 0.5 // updates motor PID values. Notice space delimited
  W6.28           // updates wheels rads/sec


*/

#ifndef __BEAKER_PITALK__
#define __BEAKER_PITALK__

#include "./wheels.h"
#include "./imu.h"

class PiTalk {
  private:

  std::string str; // holds incoming serial messages from Pi

  Wheels *wheels;
  Imu *my_imu;
  void (*callbackFunction) (char, std::string) = nullptr;

  // The IMU's theta value does not deliver a perfeft 0(zero)
  // when robot is balanced. This allows you to update.
  void updateThetaOffset(std::string message){
    float newOffset = String(message.c_str()).toFloat();
    my_imu->setThetaOffset(newOffset);
  }

  // This is the main method for controlling Beaker, if the algorithm
  // is housed in the Raspberry Pi (such as with Reinforcement Learning)
  void updateWheelRadsPerSec(std::string message){
    float newRadsPerSec = String(message.c_str()).toFloat();
    wheels->updateRadsPerSec(newRadsPerSec);
  }

  // Update the low-level PID values for the motors. This should
  // not be needed when working with various algorithms. You
  // should prob just leave the values found in constants.h.
  void updateMotorPids(std::string message){
    float newPIDs[3] = {};
    stringToFloats(message, newPIDs);
    wheels->updatePids(newPIDs[0], newPIDs[1], newPIDs[2]);
  }

  // Try for the universal commands, and if nothing matches,
  // hit the callback for the algorithm-specific commands
  void handleCommandFromPi(std::string command_plus_message){
    std::string message = command_plus_message.substr(1);
    char command;
    command = command_plus_message[0];

    switch(command){
      case 'B': updateThetaOffset(message); break;
      case 'M': updateMotorPids(message); break;
      case 'W': updateWheelRadsPerSec(message); break;
      default : callbackFunction(command, message); break;
    }
  }

  public:

  PiTalk() {
    Serial2.begin(115200);
    while (!Serial2) {;}
    str = "";
  }

  void setup(Wheels *w){
    wheels = w;
  }

  void setup(Wheels *w, Imu *i, void (*callbk) (char, std::string)){
    wheels = w;                 // regardless of algorithm, wheels is needed
    my_imu = i;                 // regardless of algorithm, IMU is needed
    callbackFunction = callbk;  // algorithm specific continuation of switch statement
  }

  // update data from Pi.
  // If full command is here, process it.
  void checkForPiCommand(){
    char tmp_char;

    while(Serial2.available()) {
      tmp_char = Serial2.read();
      if(tmp_char == '!'){
        handleCommandFromPi(str);
        str = "";
      }else{
        str += tmp_char;
      }
    }
  }

  // copy(is, eos, floats) was crashing. not sure why...
  // it was only crashing once I used this method via callback.
  // when I used it in main, 'copy' worked fine.  ¯\_(ツ)_/¯
  void stringToFloats(std::string &message, float *floats){
    std::istringstream ss( message );
    std::istream_iterator <float> is( ss );
    std::istream_iterator <float> eos;
    for(unsigned int i = 0; i<sizeof(*floats);i++){
      if (is!=eos) floats[i] = *is;
      is++;
    }
  }
};

#endif
