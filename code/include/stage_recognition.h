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

struct quat
{
  float r;
  float i;
  float j;
  float k;

  quat(float _r, float _i, float _j, float _k){
    r=_r;
    i=_i;
    j=_j;
    k=_k;
  }
  quat(){
    r=0.0;
    i=0.0;
    j=0.0;
    k=0.0;
  }

  //prints a quat to Serial
  void print(const char* name, bool lbreak){
    Serial.print(name);
    Serial.print(": r: ");
    Serial.print(r);
    Serial.print(", i: ");
    Serial.print(i);
    Serial.print(", j: ");
    Serial.print(j);
    Serial.print(", k: ");
    Serial.print(k);
    if(lbreak)
      Serial.println();
  }

  //inverts a quat
  quat invert(){
    return quat(r,-i,-j,-k);
  }

  //the hamilton product of two quaternions, equates to q1*q2
  quat multiply(quat q2){
    float _r = r*q2.r - i*q2.i - j*q2.j - k*q2.k;
    float _i = r*q2.i + i*q2.r + j*q2.k - k*q2.j;
    float _j = r*q2.j - i*q2.k + j*q2.r + k*q2.i;
    float _k = r*q2.k + i*q2.j - j*q2.i + k*q2.r;
    return quat(_r,_i,_j,_k);
  }
};

struct vec3
{
  float x;
  float y;
  float z;

  vec3(float _x, float _y, float _z) {
    x=_x;
    y=_y;
    z=_z;
  }
  vec3(){
    x=0.0;
    y=0.0;
    z=0.0;
  }

  //prints a vec3 to serial
  void print(const char* name, bool lbreak){
    Serial.print(name);
    Serial.print(": x: ");
    Serial.print(x);
    Serial.print(": y: ");
    Serial.print(y);
    Serial.print(": z: ");
    Serial.print(z);
    if(lbreak)
      Serial.println();
  }

  //rotates a vec3 using a quat
  vec3 rotate(quat rotVec){
    quat aQuat = quat(0,x,y,z);
    quat rotVecInv = rotVec.invert();

    quat t = rotVec.multiply(aQuat);
    quat rotAcc = t.multiply(rotVecInv);
    return vec3(rotAcc.i, rotAcc.j, rotAcc.k);
  }
};

struct telemetry
{
  //the LOCAL acceleration of the rocket
  vec3 acc;                   //3*4 bytes

  //the GLOBAL acceleration of the rocket, highly untested
  vec3 rotAcc;               //3*4bytes THIS WAS NOT IN THE CODE THAT FLEW

  //the rotation of the rocket
  quat rot;                   //4*4 bytes

  //did the bno reset this iteration?
  bool bnoReset;              //1 (+3 alignment) bytes
  //A bitmask for missed readings withe bno. A 1 in the first place means 
  //we missed acc, and in the second place means a missed rot reading
  int bnoMissed;              // 4 bytes

  int direction;              //4 bytes
  float alt;                  //4 bytes
  int parachute_state;        //4 bytes
  float temp ;                //4 bytes
  float pres;                 //4 bytes
  float base_pres;            //4 bytes
  long flight_time;           //4 bytes
  long time;                  //4 bytes
                              //in total: 80 bytes (20 words) (68 bytes (17 words) at launch of Mk IIa)
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
