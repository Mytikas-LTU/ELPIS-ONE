Skip to content
Search or jump to…
Pull requests
Issues
Codespaces
Marketplace
Explore
 
@Bullizz 
Mytikas-LTU
/
Mytikas
Private
Code
Issues
Pull requests
1
Actions
Projects
1
Security
Insights
Settings
Mytikas/recovery/include/stage_recognition.h
@Bullizz
Bullizz Recovery _Code
…
Latest commit fce0166 on Feb 2
 History
 1 contributor
47 lines (41 sloc)  1.16 KB

#include <Arduino.h>

/**
 *Author: Simon Birgersson & Albin Malmqvist
 *Justification :
 *  Contains descriptive comments for every function in the cpp file
 **/

#ifndef __header_h
#define __header_h

enum STATE // Defines stages
{
  launch_pad = 1,
  quick_ascent = 2,
  slow_ascent = 3,
  apogee = 4,
  quick_descent = 5,
  slow_descent = 6,
  touch_down = 7,
  FALSE = 0
};

struct telemetry
{
  float acc;
  float vel;
  float height;
  int parachute_state;
};

/**
 * function that generates values for acc, vel and height to check against stage_recognition
 * takes the ptr telemetry as input to give values variables declared in the struct above
 * i is used as so called tick, a tick can be seen as a time-unit during the fligt
 * returns 0, only used when simulation should stop
 **/
int gen_dummy_data(telemetry *telemetry, float i, int current_state);

/**
 * function that declares rockets stage in flight
 * input ptr telemtry gives the values of acc, vel, height and parachute_status
 * previous state is the latest stage stage_of_flight declared
 * returns the rockets current stage in its flight
 **/
int state_of_flight(telemetry *telemetry, int previous_state);
#endif
