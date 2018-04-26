/*
  Beaker's Arduino communicates with the Raspberry Pi over Serial.

  Communication is bidirectional.

  This library facilitates.

*/

#ifndef __BEAKER_PITALK__
#define __BEAKER_PITALK__

#include "./wheels.h"
#include "./imu.h"

class PiTalk {
  private:

  // vars to hold incoming serial messages from Pi
  std::string str;

  Wheels *wheels;
  Imu *my_imu;
  void (*callbackFunction) (char, std::string) = NULL;

  void updateThetaOffset(std::string message){
    float newOffset = String(message.c_str()).toFloat();
    my_imu->setThetaOffset(newOffset);
  }

  // Try for the universal commands, and if nothing matches,
  // hit the callback for the algorithm-specific commands
  void handleCommandFromPi(std::string command_plus_message){
    std::string message = command_plus_message.substr(1);
    char command;
    command = command_plus_message[0];

    switch(command){
      case 'B': updateThetaOffset(message); break;
      default : callbackFunction(command, message); break;
    }
  }

  public:

  PiTalk() {
    Serial2.begin(115200);
    while (!Serial2) {;}
    str = "";
  }

  void setup(Wheels *w, Imu *i, void (*callbk) (char, std::string)){
    wheels = w;
    my_imu = i;
    callbackFunction = callbk;
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
};

#endif
