#include "../MyDebug/MyDebug.h"
#include "Boiler.h"
#include <stdio.h>
#include <OneWire.h>
#include <DallasTemperature.h>

MyDebug BoilerDebug("BOILER",DABUGE_ACT);

OneWire oneWire(ROOM_TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

Boiler::Boiler()
{

}

void Boiler::Init()
{
  sensors.begin();
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); //switch to async mode

  this->t = sensors.getTempCByIndex(0);
  this->t_last = this->t;
  this->ts = millis();
  this->lastExtTempSet = -extTempTimeout_ms;
}

float Boiler::getInternalTemp() 
{
  internalTemp = sensors.getTempCByIndex(0);
  return internalTemp + internalTempCorrect;
}

void Boiler::loop()
{
  unsigned long timeNow = millis();
  // переодический запрос температуры с встроенного датчика температуры
  if (timeNow - this->lastIntTempSet > intTempTimeout_ms) 
  {
    this->lastIntTempSet = timeNow;
    sensors.requestTemperatures(); //async temperature request
  }
  // если давно не приходило обновления температуры с внешнего датчика
  if (timeNow - this->lastExtTempSet > extTempTimeout_ms)
  {
    this->t = this->internalTemp + this->internalTempCorrect;
    this->workTempIsExternal = false;
  }
  else 
  {
    this->t = this->externalTemp;
    this->workTempIsExternal = true;
  }
}

void Boiler::SetExtTemp (float temp)
{
  this->externalTemp = temp;
  this->lastExtTempSet = millis();
}

float Boiler::Pid(float sp, float pv, float pv_last, float& ierr, float dt) 
{
  float KP = 10;                      // установка значений коэффициентов - пропорциональный
  float KI = 0.02;                    // установка значений коэффициентов - интегрирующий
  float oplo = OPLO;                  // установка нижней границы регулятора
  float error = sp - pv;              // вычислим текущую ошибку регулятора
  ierr = ierr + KI * error * dt;      // вычислим интегрирующую ошибку
  //float dpv = (pv - pv_last) / dt;  // вычислим диф. ошибку
  // вычисление значений регулятора
  float P = KP * error;               // пропорциональная состовляющая
  float I = ierr;                     // интегрирующая состовляющая
  float op = P + I;                   // результат

  if ((op < oplo) || (op > this->ophi)) 
  {
    I = I - KI * error * dt;              // не накапливаем ошибку если в клиппенге
    op = max(oplo, min(this->ophi, op));  // выходное значение с учетом ограничений
  }
  ierr = I;                           // сохраним интегрирующую ошибку по указателю
  String messagetemp = "sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I);
  BoilerDebug.Message(messagetemp.c_str());
  return op;
}

float Boiler::Pid()
{
    float outPid;   // выходное значение регулятора
    float dt;       // дельта времени для вычисления
    // вычислим дельту времени между вычислениями используя системный таймер
    unsigned long new_ts = millis();
    dt = (new_ts - this->ts) / 1000.0;
    // сохраним текущее время для следующего вычисления
    this->ts = new_ts;
    // запустим расчет ПИД регулятора
    outPid = Pid(this->sp,this->t,this->t_last,this->ierr,dt);
    // сохраним текущую температуру для следующего вычисления
    this->t_last = this->t;
    return outPid;
}

void Boiler::CheckConfig () 
{
  if ((this->ophi > OPHIMAX) || (this->ophi < OPLO)) this->ophi = OPHIDEF;
  if ((this->sp > SPMAX) || (this->sp < SPMIN)) this->sp = SPDEF;
}