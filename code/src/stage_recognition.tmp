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

// function that declares rockets stage in flight depending on the generated values and its previous stage
int state_of_flight(telemetry *telemetry, int previous_state)
{
  if (previous_state == launch_pad && telemetry->acc <= 10 && !telemetry->parachute_state)
    return launch_pad;
  else if (previous_state == launch_pad && telemetry->acc > 10 && !telemetry->parachute_state)
    return quick_ascent;
  else if (previous_state == quick_ascent && telemetry->acc >= 0 && !telemetry->parachute_state)
    return quick_ascent;
  else if (previous_state == quick_ascent && telemetry->vel > 0 && telemetry->acc < 0 && !telemetry->parachute_state)
    return slow_ascent;
  else if (previous_state == slow_ascent && telemetry->vel > 0 && telemetry->acc < 0 && !telemetry->parachute_state)
    return slow_ascent;
  else if (previous_state == slow_ascent && (-1.5 < telemetry->vel && telemetry->vel < 1.5) && telemetry->acc < 0 && !telemetry->parachute_state)
    return apogee;
  else if (previous_state == apogee && telemetry->vel < 0 && telemetry->acc < 0 && !telemetry->parachute_state)
    return quick_descent;
  else if (previous_state == quick_descent && telemetry->vel < 0 && telemetry->acc < 0 && !telemetry->parachute_state)
    return quick_descent;
  else if (previous_state == quick_descent && telemetry->vel < 0 && telemetry->acc < 0 && telemetry->parachute_state) // parachute released, quick_descent -> slow_descent
    return slow_descent;
  else if (previous_state == slow_descent && telemetry->vel < 0 && telemetry->acc < 0 && telemetry->parachute_state)
    return slow_descent;
  else if (previous_state == slow_descent && (-1 < telemetry->vel && telemetry->vel < 1) && (-1 < telemetry->acc && telemetry->acc < 1) && telemetry->parachute_state)
    return touch_down;
  else if (previous_state == touch_down)
    return touch_down;

  else
    return FALSE; // error
}
