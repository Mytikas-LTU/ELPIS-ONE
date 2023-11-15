#include <Arduino.h>
#include "stage_recognition.h"
#define Enable_DummyData 0


// function that generates values for acc, vel and height to check against stage_recognition
#if Enable_DummyData
int gen_dummy_data(telemetry *telemetry, float i, int current_state)
{
  float dt = 0.001; // number of time-unit a loop represents

  if (i < 5 / dt) // launch_pad
  {
    telemetry->acc_globZ = 0;
    telemetry->height = 0;
    telemetry->parachute_state = 0;
    telemetry->vel = 0;
  }
  else if ((5 / dt <= i) && (i <= 8.55 / dt)) // quick_ascent
    telemetry->acc_globZ = random(8000, 10000) / 100.0;

  else if (i > 8.55 / dt && telemetry->vel > 0 && !telemetry->parachute_state) // slow_ascent, g + air_resistance
    telemetry->acc_globZ = random(-5000, -1000) / 100.0;

  else if (i > (8.55 / dt) && telemetry->vel < 0 && !telemetry->parachute_state) // quick_descent, g
    telemetry->acc_globZ = -9.82;

  else if (i > 8.55 / dt && telemetry->height > 0 && telemetry->parachute_state) // after parachute, slow_descent
  {
    telemetry->acc_globZ = random(-201, -1) / 100.0;
    telemetry->vel = random(-301, -149) / 100.0; // big down_vel because code takes to much memory otherwise
  }

  else if (i > 8.55 / dt && (-10 <= telemetry->height && telemetry->height <= 10) && telemetry->parachute_state) // touchdown
  {
    telemetry->acc_globZ = 0;
    telemetry->height = 0;
    telemetry->vel = 0;

    return 0;
  }

  telemetry->vel += telemetry->acc_globZ * dt;
  telemetry->height += telemetry->vel * dt;

  if (current_state == quick_descent && telemetry->vel <= -10 && telemetry->acc_globZ < 0)
    telemetry->parachute_state = 1;

  return 0;
}
#endif


int approx_direction(float *presArr, float basePres ){
  float diff[50];
  float sum = 0;
  for ( i = 0; i < 50; i++)
  {
    diff[i] = presArr[0] - presArr[i];
    sum+=diff[i];
  }
  float median = sum/50;

  if (median < basePres * 1.005 && median > basePres * 0.995) //only works at ground level 
  {
    return flat; //2
  }
  else if (median < 0)
  {
    return upp; //1
  }
  else if (median > 0)
  {
    return down; //3
  }
   
}


// function that declares rockets stage in flight depending on the generated values and its previous stage

int state_of_flight_func(telemetry *flight_data, int previous_state)
{
  if (((previous_state == launch_pad) + (direction == 3) + (-1 < flight_data->acc_globZ && flight_data->acc_globZ < 1) + (!flight_data->parachute_state)) >= 3 )
    return launch_pad;
  else if (((previous_state == launch_pad) + (flight_data->acc_globZ > 10) + (!flight_data->parachute_state)) >= 2)
    return quick_ascent;
  else if (((previous_state == quick_ascent) +  (flight_data->acc_globZ >= 0) + (!flight_data->parachute_state)) >= 2 )
    return quick_ascent;
  else if (((previous_state == quick_ascent) + (direction ==1 > 0) + t(elemetry->acc_globZ < 0) + (!flight_data->parachute_state)) >= 3)
    return slow_ascent;
  else if (((previous_state == slow_ascent) +  (direction ==1) + (flight_data->acc_globZ < 0) +  (!flight_data->parachute_state)) >=3)
    return slow_ascent;
  else if (((previous_state == slow_ascent) + (direction == 2) + (flight_data->acc_globZ < 0) + (!flight_data->parachute_state)) >= 3)
    long fallTime = flight_data->time;  
    return quick_descent;
  else if (((previous_state == quick_descent) + (direction == 2)  + (flight_data->acc_globZ < 0) + (!flight_data->parachute_state)) >= 3) // parachute released, quick_descent -> slow_descent
    if (falltime - flight_data->time >= 1.5 && chute_check(&flight_data))
    {
      flight_data->parachute_state = 1; 
      // release parachute;
       return slow_descent;
    }
    else{return quick_descent;}
  else if (((previous_state == slow_descent)  + (direction == 2) + (flight_data->acc_globZ < 0) + (flight_data->parachute_state)) >= 3)
    return slow_descent;
  else if (((previous_state == slow_descent) + (direction == 3) + (-1 < flight_data->acc_globZ && flight_data->acc_globZ < 1) + (flight_data->parachute_state)) >=3)
    return touch_down;
  else if (previous_state == touch_down)
    return touch_down;
  else
    return FALSE; // error
}


void emergency_chute(telemetry *flight_data)
{
  if (flight_data->alt < 100 && flight_data->direction ==3 && !flight_data->parachute_state )
  {
    flight_data->parachute_state = 1; 
    //release parachute
  }
  else if(flight_data->alt < 0 && !flight_data->parachute_state && (flight_data->state_of_flight == launch_pad || flight_data->alt < -2))
  {
    flight_data->parachute_state = 1; 
    //release parachute
  }
}




