#include <Arduino.h>
#include <FUTABA_SBUS.h>
// #include <Streaming.h>

FUTABA_SBUS sBus;

void setup() {

  //Set pinout
  pinMode(8, OUTPUT);
  Serial.begin(9500);
  sBus.begin();
}

int i = 0;

void loop() {
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    sBus.UpdateChannels();
    sBus.toChannels = 0; 

    if (sBus.channels[4] > 500 || sBus.channels[7] > 500){
      digitalWrite(8, HIGH);
    }
    else{
      digitalWrite(8, LOW);
    }
  }
} 