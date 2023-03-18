/*************************************************************
  This example runs directly on ESP8266 chip.

  Please be sure to select the right ESP8266 module
  in the Tools -> Board -> WeMos D1 Mini

  Adjust settings in Config.h before run
 *************************************************************/

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
#include "LittleFS.h" // LittleFS is declared
#include "GyverTimer.h"
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
       op = 75; //PID controller output
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
  float ophi = 75;
  float oplo = 20;
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
  //Set/Get Boiler Status
  //bool enableHotWater = true;
  bool enableCooling = false;
  unsigned long response = ot.setBoilerStatus(heatingEnabled, enableHotWater, enableCooling);
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

  //snprintf (msg, MSG_BUFFER_SIZE, "%s", String(op).c_str());
  
  client.publish(Topic(TEMP_BOILER_TARGET_GET_TOPIC).c_str(), String(op).c_str());

  //snprintf (msg, MSG_BUFFER_SIZE, "%s", String(t).c_str());
  client.publish(Topic(CURRENT_TEMP_GET_TOPIC).c_str(), String(t).c_str());

  //snprintf (msg, MSG_BUFFER_SIZE, "%s", String(bt).c_str());
  client.publish(Topic(TEMP_BOILER_GET_TOPIC).c_str(), String(bt).c_str());

  //snprintf (msg, MSG_BUFFER_SIZE, "%s", String(sp).c_str());
  client.publish(Topic(TEMP_SETPOINT_GET_TOPIC).c_str(), String(sp).c_str());

  //snprintf (msg, MSG_BUFFER_SIZE, "%s", heatingEnabled ? "heat" : "off");
  client.publish(Topic(MODE_GET_TOPIC).c_str(), heatingEnabled ? "heat" : "off");

  //snprintf (msg, MSG_BUFFER_SIZE, "%s", enableHotWater ? "on" : "off");
  client.publish(Topic(MODE_HOT_WATER_GET_TOPIC).c_str(), enableHotWater ? "on" : "off");

  client.publish(Topic(TEMP_HOT_WATER_GET_TOPIC).c_str(), String(wt).c_str());

  //snprintf (msg, MSG_BUFFER_SIZE, "%s", String(boiler_Fault).c_str());
  client.publish(Topic(BOILER_FAULT_TOPIC).c_str(), String(boiler_Fault).c_str());

  //snprintf (msg, MSG_BUFFER_SIZE, "%s", String(boiler_Modulation).c_str());
  client.publish(Topic(BOILER_MODULATION_TOPIC).c_str(), String(boiler_Modulation).c_str());

  //snprintf (msg, MSG_BUFFER_SIZE, "%s", String(internalTempCorrect).c_str());
  client.publish(Topic(INTERNAL_TEMP_CORRECT_GET_TOPIC).c_str(), String(internalTempCorrect).c_str());

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


void callback(char* topic, byte* payload, unsigned int length) {
  const String topicStr(topic);
 
  String payloadStr = convertPayloadToStr(payload, length);

  if (topicStr == setpointSetTopic) {
    Serial.println("Set target temperature: " + payloadStr);
    sp = payloadStr.toFloat();
  }
  else if (topicStr == currentTempSetTopic) {
    t = payloadStr.toFloat();
    lastTempSet = millis();
  }
  else if (topicStr == setCorrectInternalTempTopic) {
    internalTempCorrect = payloadStr.toFloat();
    Serial.println("Set correct: " + String(internalTempCorrect));
  }
  else if (topicStr == tempSetHotWaterTopic) {
    t_hot_water = payloadStr.toFloat();
    Serial.println("Hot water temperature: " + String(t_hot_water));
  }
  else if (topicStr == modeSetTopic) {
    Serial.println("Set mode: " + payloadStr);
    if (payloadStr == "heat")
      heatingEnabled = true;
    else if (payloadStr == "off")
      heatingEnabled = false;
    else
      Serial.println("Unknown mode");
  }
  else if (topicStr == modeSetHotWaterTopic) {
    if (payloadStr == "on")
      enableHotWater = true;
    else if (payloadStr == "off")
      enableHotWater = false;
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

  listDir("/");

  if (!(myserv.loadConfig(PATH_SETJSON)))
  {
    myserv.saveConfig(PATH_SETJSON);
  }

  //mwifi.SetConfig("ZosiWiFi","Zosi-3422");
  mwifi.SetConfig(myserv.getConfig().SSID,myserv.getConfig().PSW);
  mwifi.SetHostname("Boiler");
  mwifi.AutoWiFi();
  //if (startWiFi()) 
  myserv.startServer();

  ArduinoOTA.setHostname("OpenTherm");
  ArduinoOTA.setPassword((const char *)"123");
  ArduinoOTA.begin();   // You should set a password for OTA. Ideally using MD5 hashes
  
/*
  int deadCounter = 30;
  while (WiFi.status() != WL_CONNECTED && deadCounter-- > 0) {
    delay(1000);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("");
    Serial.print("Connecting to " + String(ssid2));
    WiFi.begin(ssid2, pass2);
    deadCounter = 20;
    while (WiFi.status() != WL_CONNECTED && deadCounter-- > 0) {
      delay(500);
      Serial.print(".");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to " + String(ssid));
    //while (true);
  }
  else {
    Serial.println("ok");
  }
*/

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
  // Loop until we're reconnected
  //while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    const char* clientId = "opentherm-thermostattest";
    if (client.connect(clientId, mqtt_user, mqtt_password)) {
      Serial.println("ok");

      client.subscribe(Topic(TEMP_SETPOINT_SET_TOPIC).c_str());
      client.subscribe(Topic(MODE_SET_TOPIC).c_str());
      client.subscribe(Topic(CURRENT_TEMP_SET_TOPIC).c_str());
      client.subscribe(Topic(MODE_HOT_WATER_SET_TOPIC).c_str());
      client.subscribe(Topic(INTERNAL_TEMP_CORRECT_SET_TOPIC).c_str());
      client.subscribe(Topic(TEMP_HOT_WATER_SET_TOPIC).c_str());
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }
  //}
}

void loop()
{
  ArduinoOTA.handle();
  unsigned long now = millis();
  if (!client.connected()) {
    if (now - lastMQTTConnect > statusConnectMQTT_ms) {
      reconnect();
      lastMQTTConnect = now;
    }
  }
  else client.loop();

  
  if (now - lastUpdate > statusUpdateInterval_ms) {
    lastUpdate = now;
    myserv.handleClientServer();
    updateData();
  }
}