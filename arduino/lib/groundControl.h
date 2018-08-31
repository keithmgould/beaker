/*
    Note: Technical Debt Alert.

    This class is a ripoff of PiTalk. 

    Really there should be a parent class (called Talk)
    and both PiTalk and GroundControl should inherit. 

    The main difference is just that GroundControl does not
    use messageIds, while PiTalk does.

*/

#ifndef __BEAKER_GROUNDCONTROL__
#define __BEAKER_GROUNDCONTROL__

#include "./wheels.h"
#include "./imu.h"

class GroundControl {
  private:

  std::string str; // holds incoming serial messages from Pi

  HardwareSerial *serial;
  Wheels *wheels;
  Imu *my_imu;
  void (*callbackFunction) (char, std::string) = nullptr;

  // The IMU's theta value does not deliver a perfeft 0(zero)
  // when robot is balanced. This allows you to update.
  void updateThetaOffset(std::string message){
    float newOffset = String(message.c_str()).toFloat();
    my_imu->setThetaOffset(newOffset);

    sendToGC("Updated Theta Offset.");
  }

  // This is one of two main method for controlling Beaker, if the algorithm
  // is housed in the Raspberry Pi (such as with Reinforcement Learning)
  // NOTE: there is no response for this call. It is meant to be used
  // very frequently (if the RPI holds the main control algorithm) so
  // speed is important, and we don't want to waste time with a response.
  //
  // updates to a new rads/sec 
  void updateWheelRadsPerSec(std::string message){
    float newRadsPerSec = String(message.c_str()).toFloat();
    wheels->updateRadsPerSec(newRadsPerSec);
  }

  // This is one of two main method for controlling Beaker, if the algorithm
  // is housed in the Raspberry Pi (such as with Reinforcement Learning)
  // NOTE: there is no response for this call. It is meant to be used
  // very frequently (if the RPI holds the main control algorithm) so
  // speed is important, and we don't want to waste time with a response.
  //
  // accelerates existing rads/sec by acceleration
  void accelerateWheelRadsPerSec(std::string message){
    float acceleration = String(message.c_str()).toFloat();
    wheels->accelerateRadsPerSec(acceleration);
  }

  // Update the low-level PID values for the motors. This should
  // not be needed when working with various algorithms. You
  // should prob just leave the values found in constants.h.
  void updateMotorPids(std::string message){
    float newPIDs[3] = {};
    stringToFloats(message, newPIDs);
    wheels->updatePids(newPIDs[0], newPIDs[1], newPIDs[2]);

    sendToGC("Updated Motor PIDs.");
  }

  // Stops the wheels and sets edge counts and target rads/sec to zero.
  void reset(){
    wheels->reset();

    sendToGC("Reset robot.");
  }

  // ----------------------------------------------------
  //format of response is:
  // char 0: messageId (can be 0-9)
  // char 1: command
  // char 2-N: message
  //
  // Try for the universal commands, and if nothing matches,
  // hit the callback for the algorithm-specific commands
  void handleCommandFromPi(std::string command_plus_message){
    std::string message = command_plus_message.substr(2);
    char command;

    command = command_plus_message[0];

    switch(command){
      case 'B': updateThetaOffset(message); break;
      case 'M': updateMotorPids(message); break;
      case 'W': updateWheelRadsPerSec(message); break;
      case 'A': accelerateWheelRadsPerSec(message); break;
      case 'R': reset(); break;
      default : callbackFunction(command, message); break;
    }
  }

  public:

  GroundControl(HardwareSerial *ourSerial) {
    serial = ourSerial;
    serial->begin(115200); while (!serial) {;}
    str = "";
  }

  void setup(Imu *i, void (*callbk) (char, std::string)){
    my_imu = i;                 // regardless of algorithm, IMU is needed
    callbackFunction = callbk;  // algorithm specific continuation of switch statement
  }

  void setup(Wheels *w){
    wheels = w;
  }

  void setup(Wheels *w, Imu *i){
    wheels = w;                 // regardless of algorithm, wheels is needed
    my_imu = i;                 // regardless of algorithm, IMU is needed
  }

  void setup(Wheels *w, void (*callbk) (char, std::string)){
    wheels = w;                 // regardless of algorithm, wheels is needed
    callbackFunction = callbk;  // algorithm specific continuation of switch statement
  }

  void setup(Wheels *w, Imu *i, void (*callbk) (char, std::string)){
    wheels = w;                 // regardless of algorithm, wheels is needed
    my_imu = i;                 // regardless of algorithm, IMU is needed
    callbackFunction = callbk;  // algorithm specific continuation of switch statement
  }

  // If full command is here, process it. Otherwise, store partial message
  // and give control back to main loop.
  void checkForCommand(){
    char tmp_char;

    while(serial->available()) {
      tmp_char = serial->read();
      if(tmp_char == '!'){
        handleCommandFromPi(str);
        str = "";
      }else{
        str += tmp_char;
      }
    }
  }

  void sendToGC(char code, String message){
    String final;
    final = String(code) + message + "!";
    serial->write(final.c_str());
  }

  void sendToGC(String message){
    sendToGC('A', message);
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
