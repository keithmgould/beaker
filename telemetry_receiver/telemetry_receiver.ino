#include <SPI.h>          // for communication with radio
#include "RF24.h"         // radio library

RF24 radio(7,8); // ce / csn

String line;

void setup(){
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println("\n\nBeginning initializations...");

  // prep our line string
  line = "";

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

//bool terminated(char * text){
//  char term[] = ";";
//  char *output = NULL;
//  output = strstr (text,term);
//  return (bool) output;
//}

// 0 means ready for A
// 1 means ready for B
int state = 0;
String str_text;

void loop(){
//  Serial.print(".");
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    str_text = String(text);
    Serial.println(str_text);
    
//    if(str_text.startsWith(String("A"))){
//      line = str_text.substring(1);
//      state = 1;
//    }else if(str_text.startsWith(String("B")) && state == 1){
//      line += str_text.substring(1);
//      state = 0;
//      Serial.println(line);
//      line = "";
//    }else{
//      state = 0;
//      line = "";
//    }
  }
}
