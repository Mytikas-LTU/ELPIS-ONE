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
  upp = 1,
  flat = 2,
  down = 3
};

struct vec3
{
  float x;
  float y;
  float z;
};

struct quat
{
  float R;
  float I;
  float J;
  float K;
};

struct telemetry
{

  vec3 acc;

  // vec3 rotAcc

  quat rot;

  bool bnoReset = false;
  int direction;
  float alt;
  int parachute_state;
  float temp ;
  float pres;
  float base_pres;
  long flight_time;
  long time;
  float acc_globZ;
};

/**
 * function that generates values for acc, vel and height to check against stage_recognition
 * takes the ptr telemetry as input to give values variables declared in the struct above
 * i is used as so called tick, a tick can be seen as a time-unit during the fligt
 * returns 0, only used when simulation should stop
 **/
void gen_dummy_data(telemetry *telemetry, int i, int current_state);

/**
 * function that declares rockets stage in flight
 * input ptr telemtry gives the values of acc, vel, height and parachute_status
 * previous state is the latest stage stage_of_flight declared
 * returns the rockets current stage in its flight
 **/
int state_of_flight_func(telemetry *telemetry, int previous_state);

void emergency_chute(telemetry *flight_data,int previous_state);

int approx_direction(float *presArr, telemetry *flight_data );

#endif
