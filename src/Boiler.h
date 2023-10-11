#ifndef Boiler_h
#define Boiler_h

#include "Arduino.h"

// Temperature sensor pin
const int ROOM_TEMP_SENSOR_PIN = D3; //for Arduino, 14 for ESP8266 (D5), 18 for ESP32

#define OPLO      20
#define OPHIDEF   65
#define OPHIMAX   85
#define SPDEF     20
#define SPMIN     5
#define SPMAX     30

// константы временных интервалов
const unsigned long intTempTimeout_ms = 10 * 1000;
const unsigned long extTempTimeout_ms = 60 * 1000;

class Boiler 
{
  public: 
  Boiler();
  void Init();
  bool workTempIsExternal;  // когда работаем с данными внешних датчиков

  float internalTempCorrect = 0;  // корректировка температуры внутреннего датчика
  float internalTemp = 0; // значение температуры с внутреннего датчика
  float externalTemp = 0; // значение температуры с внешнего датчика
  float op = 75; // выходное значение PID регулятора
  float t = 18; // текущая температура для расчета
  float sp = 18; // установленное значение 
  float t_last = 0; // предыдущее значение температуры
  float ierr = 25; // интегрирующая ошибка
  float ophi = 75;  // максимальное значение температуры на выходе регулятора
  unsigned long ts = 0; // значение времени последнего вычисления 
  
  void loop();
  float getInternalTemp();
  void SetExtTemp(float temp);
  float Pid(float sp, float pv, float pv_last, float& ierr, float dt);
  float Pid();
  void CheckConfig();

  private:
  // сохранение значений времени для работы интервалов 
  unsigned long lastIntTempSet = 0;
  unsigned long lastExtTempSet = 0;
};

#endif