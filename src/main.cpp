#include <Arduino.h>
#include <Joystick.h>
#include <FUTABA_SBUS.h>
#include "utils/SBusTracker.h"
// #include <Streaming.h>


// Compile-time debug logging.
// To enable debug serial output, define the `DEBUG_LOG` macro in your build flags.
// For PlatformIO add this to your environment in `platformio.ini`:
//
// [env:your_env]
// build_flags = -DDEBUG_LOG
//
// Note: On some boards the Serial interface may conflict with USB HID functionality
// (e.g., when emulating a joystick). Only enable serial debug while testing.
#if defined(DEBUG_LOG)
#define DEBUG_BEGIN(baud) Serial.begin(baud)
#define DEBUG_PRINT(val) Serial.print(val)
#define DEBUG_PRINTLN(val) Serial.println(val)
#else
#define DEBUG_BEGIN(baud) do {} while (0)
#define DEBUG_PRINT(val) do {} while (0)
#define DEBUG_PRINTLN(val) do {} while (0)
#endif

#define MIN_SIGNAL 190
#define MAX_SIGNAL 1790

#define ACTIVE_SIGNAL 1792
#define RELEASED_SIGNAL 192
#define MAJORITY_THREASH ( (HISTORY_SIZE / 2 + 1) * ACTIVE_SIGNAL) / HISTORY_SIZE

#define XAXIS_CHANNEL 3
#define YAXIS_CHANNEL 2
#define RX_CHANNEL 0
#define RY_CHANNEL 1
#define L_TRI_SWITCH_CHANNEL 5
#define R_TRI_SWITCH_CHANNEL 6
#define LBUTTON_CHANNEL 4
#define RBUTTON_CHANNEL 7
#define SE_BUTTON_CHANNEL 8

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
long se_est_buffer[FILTER_WINDOW_SIZE] = {0};

int l_est_index = 0;  // Track current index for left filter
int r_est_index = 0;  // Track current index for right filter
int se_est_index = 0;

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
  11, 0,                  // Button Count, Hat Switch Count
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
SBusTracker SEButTracker;

void setup() {

  //Set pinout
  pinMode(8, OUTPUT);
  // Initialize Serial only when debug logging is enabled
  DEBUG_BEGIN(115200);
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
  SEButTracker.add(sBus.channels[SE_BUTTON_CHANNEL]);
}

int get_button_state(int estimated) {
  if (estimated > MAJORITY_THREASH) {
    // return ACTIVE_SIGNAL;
    return RELEASED_SIGNAL;
  } else {
    // return RELEASED_SIGNAL;
    return ACTIVE_SIGNAL;
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
    Joystick.setButton(0,get_button_state(lButTracker.get_estimated()));
    Joystick.setButton(1,get_button_state(rButTracker.get_estimated()));

    // Apply the moving average filter
    // long l_est = lTriSwitchTracker.get_estimated();
    // long r_est = rTriSwitchTracker.get_estimated();
    long l_est = moving_average(l_est_buffer, l_est_index, FILTER_WINDOW_SIZE, lTriSwitchTracker.get_estimated());
    long r_est = moving_average(r_est_buffer, r_est_index, FILTER_WINDOW_SIZE, rTriSwitchTracker.get_estimated());
    int se_est = moving_average(se_est_buffer, se_est_index, FILTER_WINDOW_SIZE, SEButTracker.get_estimated());

    // Combine the two switch modes (L_TRI_SWITCH and R_TRI_SWITCH) into a single index (0-8)
    int modeIndex = (static_cast<int>(Map.getTriSwitchMode(l_est)) * 3) + static_cast<int>(Map.getTriSwitchMode(r_est));

    // Use the modeIndex to select the corresponding button state
    switch (modeIndex)
    {
      case 0: // L: DOWN, R: DOWN
        Joystick.setButton(2, get_button_state(se_est));
        break;
      case 1: // L: DOWN, R: MID
        Joystick.setButton(3, get_button_state(se_est));
        break;
      case 2: // L: DOWN, R: UP
        Joystick.setButton(4, get_button_state(se_est));
        break;
      case 3: // L: MID, R: DOWN
        Joystick.setButton(5, get_button_state(se_est));
        break;
      case 4: // L: MID, R: MID
        Joystick.setButton(6, get_button_state(se_est));
        break;
      case 5: // L: MID, R: UP
        Joystick.setButton(7, get_button_state(se_est));
        break;
      case 6: // L: UP, R: DOWN
        Joystick.setButton(8, get_button_state(se_est));
        break;
      case 7: // L: UP, R: MID
        Joystick.setButton(9, get_button_state(se_est));
        break;
      case 8: // L: UP, R: UP
        Joystick.setButton(10, get_button_state(se_est));
        break;
      default:
        // Handle any unexpected cases, though they shouldn't occur with the above logic.
        break;
    }

    // Print the estimated values for debugging
    // DEBUG_PRINT("L Tri Switch Mode: ");
    // DEBUG_PRINT(Map.getTriSwitchMode(l_est));
    // DEBUG_PRINT(", ");
    // DEBUG_PRINT(l_est);
    // DEBUG_PRINT(" | R Tri Switch Mode: ");
    // DEBUG_PRINT(Map.getTriSwitchMode(r_est));
    // DEBUG_PRINT(", ");
    // DEBUG_PRINTLN(r_est);
    DEBUG_PRINTLN(modeIndex);
    DEBUG_PRINTLN(se_est);
    
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