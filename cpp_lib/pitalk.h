/*
  Beaker's Arduino communicates with the Raspberry Pi over Serial.

  Communication is bidirectional.

*/

#ifndef __BEAKER_PITALK__
#define __BEAKER_PITALK__

class PiTalk {
  private:

// vars to hold incoming serial messages from Pi
char tmp_char;
std::string str;


  // update data from Pi.
  // If full command is here, process it.
  void checkForPiCommand(){
    while(Serial2.available()) {
      tmp_char = Serial2.read();
      if(tmp_char == '!'){
        handle_command_from_pi(str);
        str = "";
      }else{
        str += tmp_char;
      }
    }
  }


  public:

  void updateFloat(std::string message){
  Serial.print("Updating Theta Bias to ");
  Serial.println(message.c_str());
  thetaBias = String(message.c_str()).toFloat();
}

  PiTalk() {
    Serial2.begin(115200);
    while (!Serial2) {;}
  }

  bool isCommandFromPi() {
    return false;
  }

  std::string fetchCommandFromPi(){
    return str;
  }
};

#endif
