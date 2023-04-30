#include <Arduino.h>
#include <FUTABA_SBUS.h>
#include <TimerOne.h>
// #include <Streaming.h>

FUTABA_SBUS sBus;
volatile bool noConnection = 0;

void write_to_pins(uint8_t val){
  digitalWrite(5, val);
  digitalWrite(7, val);
  //digitalWrite(8, val);
}

void safetyInterrupt(void){
  if(noConnection){
    write_to_pins(HIGH);
  }
  else{
    write_to_pins(LOW);
  }
  noConnection = true;
}

void setup() {
  write_to_pins(OUTPUT);

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
      write_to_pins(HIGH);
      noConnection = true;

    }
    else{
      write_to_pins(LOW);
      noConnection = false;

    }
  }
} 