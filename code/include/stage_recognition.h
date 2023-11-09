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
  quick_descent = 4,
  slow_descent = 5,
  touch_down = 6,
  FALSE = 0
};

enum Direction // Defines directions
{
  upp = 1
  flat = 2
  down = 3
  FALSE = 0
}

struct telemetry
{
  float accX;
  float accY;
  float accZ;
  float acc_globZ;
  int direction;
  float alt;
  int parachute_state;
  float temp;
  float pres;
  //float gravX;
  //float gravY;
  //float gravZ;
  long time; 

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
int state_of_flight_func(telemetry *telemetry, int previous_state);
#endif