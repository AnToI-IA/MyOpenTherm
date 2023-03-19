
#define FS_NO_GLOBALS
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <OpenTherm.h>

#include <MyDebug.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <MyWiFI.h>
#include "LittleFS.h"
#include "main.h"
#include "MyServer.h"

MyWiFI mwifi;
MyServer myserv;

const unsigned long intTempTimeout_ms = 10 * 1000;
const unsigned long extTempTimeout_ms = 60 * 1000;
const unsigned long statusUpdateInterval_ms = 5000;
const unsigned long statusConnectMQTT_ms = 5000;
float internalTempCorrect = 0;
float inttemp = 0;

float  sp = 18, //set point
       t = 18, //current temperature
       t_last = 0, //prior temperature
       ierr = 25, //integral error
       dt = 0, //time between measurements
       op = 75, //PID controller output
       ophi = 75;
float t_hot_water;
unsigned long ts = 0, new_ts = 0; //timestamp
unsigned long lastUpdate = 0;
unsigned long lastMQTTConnect = 0;
unsigned long lastTempSet = 0;
unsigned long lastIntTempSet = 0;

bool heatingEnabled = true;
bool enableHotWater = false;
unsigned char boiler_Fault = 0;
float boiler_Modulation = 0;

//#define MSG_BUFFER_SIZE  (50)
//char msg[MSG_BUFFER_SIZE];

bool needSave = false;

OneWire oneWire(ROOM_TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
OpenTherm ot(OT_IN_PIN, OT_OUT_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

void ICACHE_RAM_ATTR handleInterrupt() {
  ot.handleInterrupt();
}

String Topic (const char* st )
{
  String st1 = TOPIC; 
  String st2 = st; 
  return st1 + st2;
}

float getTemp() {
  inttemp = sensors.getTempCByIndex(0);
  unsigned long now = millis();
  if (now - lastTempSet > extTempTimeout_ms)
    return inttemp + internalTempCorrect;
  else
    return t;
}

float pid(float sp, float pv, float pv_last, float& ierr, float dt) {
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
  if ((op < oplo) || (op > ophi)) {
    I = I - KI * error * dt;
    // clip output
    op = max(oplo, min(ophi, op));
  }
  ierr = I;
  Serial.println("sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I));
  return op;
}

// This function calculates temperature and sends data to MQTT every second.
void updateData()
{
  unsigned long response;
  response = ot.setBoilerStatus(heatingEnabled, enableHotWater);
  
  OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();

  boiler_Fault = ot.getFault();
  boiler_Modulation = ot.getModulation();

  if (responseStatus != OpenThermResponseStatus::SUCCESS) {
    Serial.println("Error: Invalid boiler response " + String(response, HEX));
  }

  t = getTemp();
  new_ts = millis();
  dt = (new_ts - ts) / 1000.0;
  ts = new_ts;
  if (responseStatus == OpenThermResponseStatus::SUCCESS) {
    op = pid(sp, t, t_last, ierr, dt);
    ot.setBoilerTemperature(op);
  }
  t_last = t;
  unsigned long tnow = millis();
  if (ot.isHotWaterActive(response))
  {
    ot.setDHWSetpoint(t_hot_water);
    Serial.print("Set temperature hot water: " + String(t_hot_water) + " °C ");
  } 
  if (tnow - lastIntTempSet > intTempTimeout_ms) {
    lastIntTempSet = tnow;
    sensors.requestTemperatures(); //async temperature request
  }

  Serial.print("Current temperature: " + String(t) + " °C ");
  String tempSource = (millis() - lastTempSet > extTempTimeout_ms)
                      ? "(internal sensor)"
                      : "(external sensor)";
  Serial.println(tempSource);

  if (!client.connected()) return;

  float bt = ot.getBoilerTemperature();
  float wt = ot.getDHWTemperature();
  
  client.publish(Topic(TEMP_BOILER_TARGET_GET_TOPIC).c_str(), String(op).c_str());
  client.publish(Topic(CURRENT_TEMP_GET_TOPIC).c_str(), String(t).c_str());
  client.publish(Topic(TEMP_BOILER_GET_TOPIC).c_str(), String(bt).c_str());
  client.publish(Topic(TEMP_SETPOINT_GET_TOPIC).c_str(), String(sp).c_str());
  client.publish(Topic(MODE_GET_TOPIC).c_str(), heatingEnabled ? "heat" : "off");
  client.publish(Topic(MODE_HOT_WATER_GET_TOPIC).c_str(), enableHotWater ? "heat" : "off");
  client.publish(Topic(TEMP_HOT_WATER_GET_TOPIC).c_str(), String(wt).c_str());
  client.publish(Topic(TEMP_HOT_WATER_GET_CUR_TOPIC).c_str(), String(t_hot_water).c_str());
  client.publish(Topic(BOILER_MODULATION_TOPIC).c_str(), String(boiler_Modulation).c_str());
  client.publish(Topic(INTERNAL_TEMP_CORRECT_GET_TOPIC).c_str(), String(internalTempCorrect).c_str());
  client.publish(Topic(MAX_BOILER_TEMP_GET_TOPIC).c_str(), String(ophi).c_str());
  client.publish(Topic(INT_SENSOR_TEMP_GET_TOPIC).c_str(), String(inttemp).c_str());
}

String convertPayloadToStr(byte* payload, unsigned int length) {
  char s[length + 1];
  s[length] = 0;
  for (unsigned int i = 0; i < length; ++i)
    s[i] = payload[i];
  String tempRequestStr(s);
  return tempRequestStr;
}

const String setpointSetTopic(Topic(TEMP_SETPOINT_SET_TOPIC).c_str());
const String currentTempSetTopic(Topic(CURRENT_TEMP_SET_TOPIC).c_str());
const String modeSetTopic(Topic(MODE_SET_TOPIC).c_str());
const String modeSetHotWaterTopic(Topic(MODE_HOT_WATER_SET_TOPIC).c_str());
const String tempSetHotWaterTopic(Topic(TEMP_HOT_WATER_SET_TOPIC).c_str());
const String setCorrectInternalTempTopic(Topic(INTERNAL_TEMP_CORRECT_SET_TOPIC).c_str());
const String maxBoilerTempTopic(Topic(MAX_BOILER_TEMP_SET_TOPIC).c_str());

void callback(char* topic, byte* payload, unsigned int length) {
  const String topicStr(topic);
 
  String payloadStr = convertPayloadToStr(payload, length);

  if (topicStr == setpointSetTopic) {
    if ((sp > SPMAX) || (sp < SPMIN)) Serial.print("Target temperature not change! ");
    else sp = payloadStr.toFloat();
    Serial.println("Set target temperature: " + payloadStr);
    needSave = true;
  }
  else if (topicStr == currentTempSetTopic) {
    t = payloadStr.toFloat();
    lastTempSet = millis();
  }
  else if (topicStr == setCorrectInternalTempTopic) {
    internalTempCorrect = payloadStr.toFloat();
    Serial.println("Set correct: " + String(internalTempCorrect));
    needSave = true;
  }
  else if (topicStr == tempSetHotWaterTopic) {
    t_hot_water = payloadStr.toFloat();
    Serial.println("Hot water temperature: " + String(t_hot_water));
    needSave = true;
  }
  else if (topicStr == maxBoilerTempTopic) {
    ophi = payloadStr.toFloat();
    if ((ophi > OPHIMAX) || (ophi < OPLO)) {
      Serial.print("Maximum boiler not change! ");
    }
    Serial.println("Temperature: " + String(ophi));
    needSave = true;
  }
  else if (topicStr == modeSetTopic) {
    Serial.println("Set mode: " + payloadStr);
    if (payloadStr == "heat") heatingEnabled = true;
    else if (payloadStr == "off") heatingEnabled = false;
    else Serial.println("Unknown mode");
    needSave = true;
  }
  else if (topicStr == modeSetHotWaterTopic) {
    if (payloadStr == "heat") enableHotWater = true;
    else if (payloadStr == "off") enableHotWater = false;
    else Serial.println("Unknown mode");
    needSave = true;
  }
}

void listDir(const char * dirname) {
  Serial.printf("Содержимое папки \"%s\" файловой системы SPIFS: \r\n", dirname);

  Dir root = LittleFS.openDir(dirname);

  while (root.next()) {
    File file = root.openFile("r");
    Serial.print(F("  Файл: "));
    Serial.print(root.fileName());
    Serial.print(F(";  Размер: "));
    Serial.println(file.size());
    file.close();
  }
}

void fixConfig () {
  if ((ophi > OPHIMAX) || (ophi < OPLO)) ophi = OPHIDEF;
  if ((sp > SPMAX) || (sp < SPMIN)) sp = SPDEF;

}

void setup()
{
  Serial.begin(115200);
  delay(200); 
  Serial.println();
  Serial.println("Mount LittleFS");

  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }

  //listDir("/");
  if (!(myserv.loadConfig(PATH_SETJSON)))
  {
    Serial.print("Load Config Failed. Save default");
    myserv.saveConfig(PATH_SETJSON);
  }
  fixConfig ();
  mwifi.SetConfig(myserv.getConfig().SSID,myserv.getConfig().PSW);
  mwifi.SetHostname("Boiler");
  mwifi.AutoWiFi();

  myserv.startServer();

  ArduinoOTA.setHostname("OpenTherm");
  ArduinoOTA.setPassword((const char *)"3422");
  ArduinoOTA.begin();   // You should set a password for OTA. Ideally using MD5 hashes
  

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  ot.begin(handleInterrupt);

  //Init DS18B20 sensor
  sensors.begin();
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); //switch to async mode
  t = sensors.getTempCByIndex(0);
  t_last = t;
  ts = millis();
  lastTempSet = -extTempTimeout_ms;
}


void reconnect() {
  Serial.print("Attempting MQTT connection...");
  if (client.connect(TOPIC, mqtt_user, mqtt_password)) {
    Serial.println("ok");
    client.subscribe(Topic(TEMP_SETPOINT_SET_TOPIC).c_str());
    client.subscribe(Topic(MODE_SET_TOPIC).c_str());
    client.subscribe(Topic(CURRENT_TEMP_SET_TOPIC).c_str());
    client.subscribe(Topic(MODE_HOT_WATER_SET_TOPIC).c_str());
    client.subscribe(Topic(INTERNAL_TEMP_CORRECT_SET_TOPIC).c_str());
    client.subscribe(Topic(TEMP_HOT_WATER_SET_TOPIC).c_str());
    client.subscribe(Topic(MAX_BOILER_TEMP_SET_TOPIC).c_str());
  } else {
    Serial.print(" failed, rc=");
    Serial.print(client.state());
  }
}

void loop()
{
  ArduinoOTA.handle();
  unsigned long now = millis();
  if (!client.connected()) {
    if (now - lastMQTTConnect > statusConnectMQTT_ms) {
      if (mwifi.status == Connected && mwifi.checkInternet()) reconnect();
      lastMQTTConnect = now;
    }
  }
  else client.loop();

  if (needSave) {
    myserv.saveConfig(PATH_SETJSON);
    Serial.println(" Save config!");
    needSave = false;
  }

  if (now - lastUpdate > statusUpdateInterval_ms) {
    lastUpdate = now;
    myserv.handleClientServer();
    updateData();
  }
}