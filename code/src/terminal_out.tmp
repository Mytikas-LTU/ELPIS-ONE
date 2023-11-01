#include <Arduino.h>
#include "stage_recognition.h"
#include "terminal_out.h"

/*
Author: Simon Birgersson

Purpose: Prints the values that the rocket has in each stage to terminal
*/

void init_flight_prog()
{
  telemetry telemetry;
  int previous_state = launch_pad;
  int current_state = launch_pad;
  float tick_diff = 0;   // Check the tick_diff between popping parachute and apogee A.K.A the "time"_diff
  float height_diff = 0; // Check the height_diff between popping parachute and apogee
  float i = 0.0;
  int print_parach_count = 0;
  
  while (previous_state != touch_down) // Loop that goes through each tick(second) of the flight
  {
    gen_dummy_data(&telemetry, i, current_state);                // gets flight_values
    current_state = state_of_flight(&telemetry, previous_state); // get the state_of_flight

    if (current_state == FALSE || previous_state == FALSE)
    {
      Serial.println("Error stateOfFlight");
      break;
    }
    if (current_state != previous_state)
    {
      Serial.print("\nPrevious state was: ");
      Serial.print(previous_state);
      Serial.println("");
      Serial.print("Current state is: ");
      Serial.print(current_state);
      Serial.println("");
      Serial.print("acc = ");
      Serial.print(telemetry.acc);
      Serial.println("");
      Serial.print("vel = ");
      Serial.print(telemetry.vel);
      Serial.println("");
      Serial.print("height = ");
      Serial.print(telemetry.height);
      Serial.println("");
      Serial.print("parachute_state: ");
      Serial.print(telemetry.parachute_state);
      Serial.println("");
      Serial.print("tick = "); // tick AKA time in flight
      Serial.print(i);
      Serial.println("");
      Serial.println("");
      if (current_state == apogee) // current tick and height is saved for futere difference check
      {
        tick_diff = i;
        height_diff = telemetry.height;
      }
    }

    // Printing values at parachute_pop
    if (telemetry.parachute_state && print_parach_count != 1)
    {
      Serial.print("POP PARACHUTE, values:");
      Serial.println("");
      Serial.print("acc = ");
      Serial.print(telemetry.acc);
      Serial.println("");
      Serial.print("vel = ");
      Serial.print(telemetry.vel);
      Serial.println("");
      Serial.print("height = ");
      Serial.print(telemetry.height);
      Serial.println("");
      Serial.print("tick = ");
      Serial.print(i);
      Serial.println("");
      Serial.print("height_diff = ");
      Serial.print(height_diff - telemetry.height);
      Serial.println("");
      Serial.print("tick_diff = ");
      Serial.print(i - tick_diff);
      Serial.println("");
      Serial.println("");
      print_parach_count = 1;
    }
    if (current_state == slow_descent && (int)i % 10000 == 0) // Prints some slow_descent values
    {
      Serial.print("\nacc = ");
      Serial.print(telemetry.acc);
      Serial.println("");
      Serial.print("vel = ");
      Serial.print(telemetry.vel);
      Serial.println("");
      Serial.print("height = ");
      Serial.print(telemetry.height);
      Serial.println("");
    }

    previous_state = current_state;
    i++;
  }
}
