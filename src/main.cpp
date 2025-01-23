#include <Arduino.h>
#include <Joystick.h>
#include <FUTABA_SBUS.h>
#include "utils/SBusTracker.h"
// #include <Streaming.h>

#define MIN_SIGNAL 190
#define MAX_SIGNAL 1790

#define ACTIVE_SIGNAL 1792
#define REALESED_SIGNAL 192
#define MAJORITY_THREASH ( (HISTORY_SIZE / 2 + 1) * ACTIVE_SIGNAL) / HISTORY_SIZE
#define ACTIVE_ERROR_THRESHOLD 600
#define REALESED_ERROR_THRESHOLD 400

#define XAXIS_CHANNEL 3
#define YAXIS_CHANNEL 2
#define RX_CHANNEL 0
#define RY_CHANNEL 1
#define L_TRI_SWITCH 5
#define R_TRI_SWITCH 6
#define LBUTTON_CHANNEL 4
#define RBUTTON_CHANNEL 7

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_MULTI_AXIS,
  6, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  true, true, false,   // No Rx, Ry, or Rz
  true, true,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steering

FUTABA_SBUS sBus;

SBusTracker xAxisTracker;
SBusTracker yAxisTracker;
SBusTracker rxTracker;
SBusTracker ryTracker;
SBusTracker lTriSwitchTracker;
SBusTracker rTriSwitchTracker;
SBusTracker lButTracker;
SBusTracker rButTracker;

void setup() {

  //Set pinout
  pinMode(8, OUTPUT);
  // Configure JoyStick
  Joystick.setXAxisRange(MIN_SIGNAL, MAX_SIGNAL);
  Joystick.setYAxisRange(MIN_SIGNAL, MAX_SIGNAL);
  Joystick.setRxAxisRange(MIN_SIGNAL, MAX_SIGNAL);
  Joystick.setRyAxisRange(MIN_SIGNAL, MAX_SIGNAL);
  Joystick.setThrottleRange(MIN_SIGNAL, MAX_SIGNAL);
  Joystick.setRudderRange(MIN_SIGNAL, MAX_SIGNAL);
  
  // Begin!!!
  Joystick.begin();
  sBus.begin();
}

void update_trackers(FUTABA_SBUS & sBus) {
  xAxisTracker.add(sBus.channels[XAXIS_CHANNEL]);
  yAxisTracker.add(sBus.channels[YAXIS_CHANNEL]);
  rxTracker.add(sBus.channels[RX_CHANNEL]);
  ryTracker.add(sBus.channels[RY_CHANNEL]);
  lTriSwitchTracker.add(sBus.channels[L_TRI_SWITCH]);
  rTriSwitchTracker.add(sBus.channels[R_TRI_SWITCH]);
  lButTracker.add(sBus.channels[LBUTTON_CHANNEL]);
  rButTracker.add(sBus.channels[RBUTTON_CHANNEL]);
}

int get_button_state(int estimated) {
  if (estimated > MAJORITY_THREASH) {
    return ACTIVE_SIGNAL;
  } else {
    return REALESED_SIGNAL;
  }
}

void loop() {
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    sBus.UpdateChannels();
    sBus.toChannels = 0; 

    update_trackers(sBus);

    Joystick.setXAxis(xAxisTracker.get_estimated());
    Joystick.setYAxis(yAxisTracker.get_estimated());
    Joystick.setRxAxis(rxTracker.get_estimated());
    Joystick.setRyAxis(ryTracker.get_estimated());
    Joystick.setThrottle(lTriSwitchTracker.get_estimated());
    Joystick.setRudder(rTriSwitchTracker.get_estimated());

    long l_est = lTriSwitchTracker.get_estimated();
    if (abs(l_est - ACTIVE_SIGNAL) <= ACTIVE_ERROR_THRESHOLD)
    {
      Joystick.setButton(0,get_button_state(lButTracker.get_estimated()));
    }
    else if (abs(l_est - REALESED_SIGNAL) <= REALESED_ERROR_THRESHOLD)
    {
      Joystick.setButton(4,get_button_state(lButTracker.get_estimated()));
    }
    else
    {
      Joystick.setButton(2,get_button_state(lButTracker.get_estimated()));
    }

    long r_est = rTriSwitchTracker.get_estimated();
    if (abs(r_est - ACTIVE_SIGNAL) <= ACTIVE_ERROR_THRESHOLD)
    {
      Joystick.setButton(1,get_button_state(rButTracker.get_estimated()));
    }
    else if (abs(r_est - REALESED_SIGNAL) <= REALESED_ERROR_THRESHOLD)
    {
      Joystick.setButton(5,get_button_state(rButTracker.get_estimated()));
    }
    else
    {
      Joystick.setButton(3,get_button_state(rButTracker.get_estimated()));
    }

    // Print the estimated values for debugging
    Serial.print("L Tri Switch Error: ");
    Serial.print(abs(l_est - REALESED_SIGNAL));
    Serial.print(" | R Tri Switch Error: ");
    Serial.println(abs(r_est - REALESED_SIGNAL));
    
    if (lButTracker.get_estimated() > MAJORITY_THREASH || 
        rButTracker.get_estimated() > MAJORITY_THREASH)
    {
      digitalWrite(8, HIGH);
    }
    else{
      digitalWrite(8, LOW);
    }
  }
}