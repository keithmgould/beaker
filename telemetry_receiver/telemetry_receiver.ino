#include <SPI.h>
#include "RF24.h"         // for radio controls

// PINS 6, 7 for radio ce / csn
RF24 radio(7,8);

void setup(){
  // Open console serial communications
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println("\n\nBeginning initializations...");


  // radio setup
  Serial.print("Initializing Radio...");
  const byte address[6] = "00001";
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPayloadSize(128);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Done!");
}

String line = "";

void loop(){
  if (radio.available()) {
    char text[32] = "";
    char term[] = ";";
    radio.read(&text, sizeof(text));
    // check for termination character
    char *output = NULL;
    output = strstr (text,term);
    line += String(text);
    if(output){
      line = line.substring(0,line.length() - 1);
      Serial.println(line); line = "";
    }
  }
}
