#include <Arduino.h>
#include "stage_recognition.h"
#define Enable_DummyData 1


// function that generates values for acc, vel and height to check against stage_recognition
#if Enable_DummyData
void gen_dummy_data(telemetry *telemetry, int i, int current_state)
{
  
  telemetry->base_pres = 101700;
  if (telemetry->time <= 2700)
  {
    telemetry->acc_globZ = 100;
  }
  else if (telemetry->time>= 5000)
  {
   telemetry->acc_globZ= -9.82;
  }
  
  if (i<=1)
  {
    telemetry->pres=telemetry->base_pres;
    
  }
  if (telemetry->pres>=90000 && (current_state > 1 && current_state <= 3))
  {
   telemetry->pres -= 100;
  }
  else if (telemetry->pres<=90000 && (current_state > 3))
  {
    telemetry->pres +=100;
  }
}
#endif


int approx_direction(float *presArr, telemetry *flight_data ){
  float sum = 0;
  for (int i = 0; i < 50; i++)
  {
    sum += presArr[0] - presArr[i];
    
  }
  float median = sum/50;

  if (median < 1.05 && median > -1.05) //only works at ground level 
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
  else return 0; 
}


// function that declares rockets stage in flight depending on the generated values and its previous stage

int state_of_flight_func(telemetry *flight_data, int previous_state)
{
  
  if (((previous_state == launch_pad) + (flight_data->direction == 3) + (-1 < flight_data->acc_globZ && flight_data->acc_globZ < 1) + (!flight_data->parachute_state)) >= 3 )
    return launch_pad;
  else if (((previous_state == launch_pad) + (flight_data->acc_globZ > 10) + (!flight_data->parachute_state)) >= 2)
    return quick_ascent;
  else if (((previous_state == quick_ascent) +  (flight_data->acc_globZ >= 0) + (!flight_data->parachute_state)) >= 2 )
    return quick_ascent;
  else if (((previous_state == quick_ascent) + ((flight_data->direction ==1) > 0) + (flight_data->acc_globZ < 0) + (!flight_data->parachute_state)) >= 3)
    return slow_ascent;
  else if (((previous_state == slow_ascent) +  (flight_data->direction ==1) + (flight_data->acc_globZ < 0) +  (!flight_data->parachute_state)) >=3)
    return slow_ascent;
  else if (((previous_state == slow_ascent) + (flight_data->direction == 2) + (flight_data->acc_globZ < 0) + (!flight_data->parachute_state)) >= 3)
    return quick_descent;
  else if (((previous_state == quick_descent) + (flight_data->direction == 2)  + (flight_data->acc_globZ < 0) + (!flight_data->parachute_state))) // parachute released, quick_descent -> slow_descent
    if (flight_data->flight_time - flight_data->time >= 1.5 )
    {
      flight_data->parachute_state = 1;
      // release parachute;
       return slow_descent;
    }
    else{return quick_descent;}
  else if (((previous_state == slow_descent)  + (flight_data->direction == 2) + (flight_data->acc_globZ < 0) + (flight_data->parachute_state)) >= 3)
    return slow_descent;
  else if (((previous_state == slow_descent) + (flight_data->direction == 3) + (-1 < flight_data->acc_globZ && flight_data->acc_globZ < 1) + (flight_data->parachute_state)) >=3)
    return touch_down;
  else if (previous_state == touch_down)
    return touch_down;
  else
    return FALSE; // error
}


void emergency_chute(telemetry *flight_data, int previous_state)
{
   if (flight_data->flight_time >= 15000 && flight_data->parachute_state == 0)
   {
      flight_data->parachute_state = 1;
   }
   
}




