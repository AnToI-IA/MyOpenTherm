#include "../MyDebug/MyDebug.h"
#include "Boiler.h"
#include <stdio.h>

MyDebug BoilerDebug("BOILER",DABUGE_ACT);

float Boiler::Pid(float sp, float pv, float pv_last, float& ierr, float dt) 
{
  float KP = 10;
  float KI = 0.02;
  // upper and lower bounds on heater level
  float oplo = OPLO;
  // calculate the error
  float error = sp - pv;
  // calculate the integral error
  ierr = ierr + KI * error * dt;
  // calculate the measurement derivative
  //float dpv = (pv - pv_last) / dt;
  // calculate the PID output
  float P = KP * error; //proportional contribution
  float I = ierr; //integral contribution
  float op = P + I;
  // implement anti-reset windup
  if ((op < oplo) || (op > this->ophi)) {
    I = I - KI * error * dt;
    // clip output
    op = max(oplo, min(this->ophi, op));
  }
  ierr = I;
  String messagetemp = "sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I);
  BoilerDebug.Message(messagetemp.c_str());
  return op;
}

float Boiler::Pid()
{
    float outPid;
    this->new_ts = millis();
    this->dt = (this->new_ts - this->ts) / 1000.0;
    this->ts = this->new_ts;
    outPid = Pid(this->sp,this->t,this->t_last,this->ierr,this->ts);
    this->t_last = this->t;
    return outPid;
}

void Boiler::CheckConfig () 
{
  if ((this->ophi > OPHIMAX) || (this->ophi < OPLO)) this->ophi = OPHIDEF;
  if ((this->sp > SPMAX) || (this->sp < SPMIN)) this->sp = SPDEF;
}