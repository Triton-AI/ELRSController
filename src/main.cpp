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

#define XAXIS_CHANNEL 3
#define YAXIS_CHANNEL 2
#define RX_CHANNEL 0
#define RY_CHANNEL 1
#define L_TRI_SWITCH_CHANNEL 5
#define R_TRI_SWITCH_CHANNEL 6
#define LBUTTON_CHANNEL 4
#define RBUTTON_CHANNEL 7

enum TriSwitchMode
{
    DOWN = 0,
    MID = 1,
    UP = 2,
};

struct Translation
{
    double normalize(int analogValue);
    TriSwitchMode getTriSwitchMode(int TriVal);
};

// TRANSLATION
double Translation::normalize(int analogValue) 
{
    if (analogValue > 1800)
        analogValue = 1800;
    if (analogValue < 174)
        analogValue = 174;
    analogValue -= 992;
    return analogValue >= 0 ? (double(analogValue) / (1800 - 992))
                            : (double(analogValue) / (992 - 174));
}
TriSwitchMode Translation::getTriSwitchMode(int TriVal)
{
    double TriVal_norm = normalize(TriVal);
    if (TriVal_norm < -0.5)
        return DOWN;
    else if (TriVal_norm > 0.4)
        return UP;
    else
        return MID;
}

#define FILTER_WINDOW_SIZE 10

// Buffers for the moving average filter (separate buffers for left and right)
long l_est_buffer[FILTER_WINDOW_SIZE] = {0};
long r_est_buffer[FILTER_WINDOW_SIZE] = {0};

int l_est_index = 0;  // Track current index for left filter
int r_est_index = 0;  // Track current index for right filter

long moving_average(long* buffer, int& buffer_index, int buffer_size, long new_value) {
  long sum = 0;

  // Subtract the old value and add the new value
  buffer[buffer_index] = new_value;
  buffer_index = (buffer_index + 1) % buffer_size;

  // Sum up all values in the buffer
  for (int i = 0; i < buffer_size; i++) {
    sum += buffer[i];
  }

  // Return the average
  return sum / buffer_size;
}

Translation Map;

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
  lTriSwitchTracker.add(sBus.channels[L_TRI_SWITCH_CHANNEL]);
  rTriSwitchTracker.add(sBus.channels[R_TRI_SWITCH_CHANNEL]);
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

    // Apply the moving average filter
    // long l_est = lTriSwitchTracker.get_estimated();
    // long r_est = rTriSwitchTracker.get_estimated();
    long l_est = moving_average(l_est_buffer, l_est_index, FILTER_WINDOW_SIZE, lTriSwitchTracker.get_estimated());
    long r_est = moving_average(r_est_buffer, r_est_index, FILTER_WINDOW_SIZE, rTriSwitchTracker.get_estimated());
    
    switch (Map.getTriSwitchMode(l_est))
    {
      case TriSwitchMode::UP:
        Joystick.setButton(0,get_button_state(lButTracker.get_estimated()));
        break;
      case TriSwitchMode::MID:
        Joystick.setButton(2,get_button_state(lButTracker.get_estimated()));
        break;
      case TriSwitchMode::DOWN:
        Joystick.setButton(4,get_button_state(lButTracker.get_estimated()));
        break;
    }

    switch (Map.getTriSwitchMode(r_est))
    {
      case TriSwitchMode::UP:
        Joystick.setButton(1,get_button_state(rButTracker.get_estimated()));
        break;
      case TriSwitchMode::MID:
        Joystick.setButton(3,get_button_state(rButTracker.get_estimated()));
        break;
      case TriSwitchMode::DOWN:
        Joystick.setButton(5,get_button_state(rButTracker.get_estimated()));
        break;
    }

    // Print the estimated values for debugging
    Serial.print("L Tri Switch Mode: ");
    Serial.print(Map.getTriSwitchMode(l_est));
    Serial.print(", ");
    Serial.print(l_est);
    Serial.print(" | R Tri Switch Mode: \n");
    Serial.print(Map.getTriSwitchMode(r_est));
    Serial.print(", ");
    Serial.println(r_est);
    
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