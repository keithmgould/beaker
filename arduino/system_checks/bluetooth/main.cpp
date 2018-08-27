#include "Arduino.h"
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>

/*
    This little system check is for the dedicated bluetooth connection with the Command and Control module.
    It uses Serial3 on the Arduino Mega.

    Testing Functionality:

    1. using serial terminal, verify that x's show up remotely (on other side of bluetooth), and y's show up locally (serial1)
    2. type into serial terminal, which should be converted to bluetooth, then received by Beaker. 
    Make sure to add a '!' at end of message!
*/


std::string str; // holds incoming serial messages from GroundControl

// If full command is here, process it. Otherwise, store partial message
// and give control back to main loop.
void checkForGroundControlCommand(){
  char tmp_char;

  while(Serial3.available()) {
    tmp_char = Serial3.read();
    if(tmp_char == '!'){
      Serial.println(str.c_str());
      str = "";
    }else{
      str += tmp_char;
    }
  }
}

void setup(){
  Serial.begin(115200); while (!Serial) {;}
  Serial3.begin(115200); while (!Serial) {;}
}

void loop(){
  Serial.println("y");
  Serial3.println("x");
  checkForGroundControlCommand();
  delay(500);
}