#include <Arduino.h>
#include <FUTABA_SBUS.h>
#include <TimerOne.h>
// #include <Streaming.h>

FUTABA_SBUS sBus;
volatile bool noConnection = 0;

void safetyInterrupt(void){
  if(noConnection){
    digitalWrite(8, HIGH);
  }
  else{
    digitalWrite(8, LOW);
  }
  noConnection = true;
}

void setup() {
  pinMode(8, OUTPUT);

  Timer1.initialize(100000);
  Timer1.attachInterrupt(safetyInterrupt); // check for connection every 0.1 seconds
  noConnection = true;
  //Set pinout
  sBus.begin();
}

void loop() {
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    sBus.UpdateChannels();
    sBus.toChannels = 0; 
    if (sBus.channels[4] > 500 || sBus.channels[7] > 500){
      digitalWrite(8, HIGH);
      noConnection = true;

    }
    else{
      digitalWrite(8, LOW);
      noConnection = false;

    }
  }
} 