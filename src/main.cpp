#include <Arduino.h>
#include <FUTABA_SBUS.h>
#include <TimerOne.h>
// #include <Streaming.h>

FUTABA_SBUS sBus;
volatile bool noConnection = 0;

void writeToAllPins(uint8_t val){
  digitalWrite(5, val);
  digitalWrite(7, val);
  //digitalWrite(8, val);
}

void setAllPinModes(uint8_t val){
  pinMode(5, val);
  pinMode(7, val);
  //digitalWrite(8, val);
}

void safetyInterrupt(void){
  if(noConnection){
    writeToAllPins(HIGH);
  }
  else{
    writeToAllPins(LOW);
  }
  noConnection = true;
}

void setup() {
  setAllPinModes(OUTPUT);

  Timer1.initialize(100000);
  Timer1.attachInterrupt(safetyInterrupt); // check for connection every 0.1 seconds
  noConnection = true;
  sBus.begin();
}

void loop() {
  sBus.FeedLine();

  if (sBus.toChannels == 1){
    sBus.UpdateChannels();
    sBus.toChannels = 0; 

    if (sBus.channels[4] > 500 || sBus.channels[7] > 500){
      writeToAllPins(HIGH);
      noConnection = true;
    }
    else{
      writeToAllPins(LOW);
      noConnection = false;
    }
  }
} 