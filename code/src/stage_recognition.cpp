#include <Arduino.h>
#include "stage_recognition.h"



// function that generates values for acc, vel and height to check against stage_recognition
int gen_dummy_data(telemetry *telemetry, float i, int current_state)
{
  float dt = 0.001; // number of time-unit a loop represents

  if (i < 5 / dt) // launch_pad
  {
    telemetry->acc = 0;
    telemetry->height = 0;
    telemetry->parachute_state = 0;
    telemetry->vel = 0;
  }
  else if ((5 / dt <= i) && (i <= 8.55 / dt)) // quick_ascent
    telemetry->acc = random(8000, 10000) / 100.0;

  else if (i > 8.55 / dt && telemetry->vel > 0 && !telemetry->parachute_state) // slow_ascent, g + air_resistance
    telemetry->acc = random(-5000, -1000) / 100.0;

  else if (i > (8.55 / dt) && telemetry->vel < 0 && !telemetry->parachute_state) // quick_descent, g
    telemetry->acc = -9.82;

  else if (i > 8.55 / dt && telemetry->height > 0 && telemetry->parachute_state) // after parachute, slow_descent
  {
    telemetry->acc = random(-201, -1) / 100.0;
    telemetry->vel = random(-301, -149) / 100.0; // big down_vel because code takes to much memory otherwise
  }

  else if (i > 8.55 / dt && (-10 <= telemetry->height && telemetry->height <= 10) && telemetry->parachute_state) // touchdown
  {
    telemetry->acc = 0;
    telemetry->height = 0;
    telemetry->vel = 0;

    return 0;
  }

  telemetry->vel += telemetry->acc * dt;
  telemetry->height += telemetry->vel * dt;

  if (current_state == quick_descent && telemetry->vel <= -10 && telemetry->acc < 0)
    telemetry->parachute_state = 1;

  return 0;
}

int approx_direction(float *altitude_array /*Array of the last 50 altitude measurments*/){
  float diff[50];
  for ( i = 0; i < 50; i++)
  {
    diff[i] = altitude_array[0] - altitude_array[i];
  }
  float median = sum(diff)/50;

  if (median < base_Presure * 1.005 && median > base_Presure * 0.995)
  {
    return flat; //3
  }
  else if (median > 0)
  {
    return upp; //1
  }
  else if (median < 0)
  {
    return down; //3
  }
   
}

float altitude[50]; //needed to initialize approx direction, will be pulled from measurments.
int direction = approx_direction(altitude) //will be called before state_of_flight in execution order.

// function that declares rockets stage in flight depending on the generated values and its previous stage
int state_of_flight(telemetry *telemetry, int previous_state)
{
  if (((previous_state == launch_pad) + (direction == 3) + (-1 < telemetry->acc && telemetry->acc < 1) + (!telemetry->parachute_state)) >= 3 )
    return launch_pad;
  else if (((previous_state == launch_pad) + (telemetry->acc > 10) + (!telemetry->parachute_state)) >= 2)
    return quick_ascent;
  else if (((previous_state == quick_ascent) +  (telemetry->acc >= 0) + (!telemetry->parachute_state)) >= 2 )
    return quick_ascent;
  else if (((previous_state == quick_ascent) + (direction ==1 > 0) + t(elemetry->acc < 0) + (!telemetry->parachute_state)) >= 3)
    return slow_ascent;
  else if (((previous_state == slow_ascent) +  (direction ==1) + (telemetry->acc < 0) +  (!telemetry->parachute_state)) >=3)
    return slow_ascent;
  else if (((previous_state == slow_ascent) + (direction == 2) + (telemetry->acc < 0) + (!telemetry->parachute_state)) >= 3)
    return quick_descent;
  else if (((previous_state == quick_descent) + (direction == 2)  + (telemetry->acc < 0) + (!telemetry->parachute_state)) >=3)
    return quick_descent;
  else if (((previous_state == quick_descent) + (direction == 2)  + (telemetry->acc < 0) + (telemetry->parachute_state)) >= 3) // parachute released, quick_descent -> slow_descent
    // release parachute;
    return slow_descent;
  else if (((previous_state == slow_descent)  + (direction == 2) + (telemetry->acc < 0) + (telemetry->parachute_state)) >= 3)
    return slow_descent;
  else if (((previous_state == slow_descent) + (direction == 3) + (-1 < telemetry->acc && telemetry->acc < 1) + (telemetry->parachute_state)) >=3)
    return touch_down;
  else if (previous_state == touch_down)
    return touch_down;
  else
    return FALSE; // error
}
