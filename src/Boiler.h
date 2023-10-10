#ifndef Boiler_h
#define Boiler_h

#include "Arduino.h"

#define OPLO      20
#define OPHIDEF   65
#define OPHIMAX   85
#define SPDEF     20
#define SPMIN     5
#define SPMAX     30

class Boiler {
  public: 
  float op = 75; //PID controller output
  float t = 18; //current temperature
  float dt = 0; //time between measurements
  void Test();
  float Pid(float sp, float pv, float pv_last, float& ierr, float dt);
  float Pid();
  void CheckConfig();
  //private:
  float sp = 18, //set point
    t_last = 0, //prior temperature
    ierr = 25, //integral error
    ophi = 75;
  unsigned long ts = 0, new_ts = 0; //timestamp
};

#endif