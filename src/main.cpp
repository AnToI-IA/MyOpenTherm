
#define FS_NO_GLOBALS
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <ESP8266WiFi.h>

#include <PubSubClient.h>
#include <OpenTherm.h>

#include <MyDebug.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <MyWiFI.h>
#include "LittleFS.h"
#include "main.h"
#include "MyServer.h"
#include "Boiler.h"

Boiler boiler;
MyWiFI mwifi;
MyServer myserv;



const unsigned long statusUpdateInterval_ms = 5000;
const unsigned long statusConnectMQTT_ms = 5000;
const unsigned long statusConnectWiFi_ms = 5 * 60 * 1000;




float t_hot_water;

unsigned long lastUpdate = 0;
unsigned long lastMQTTConnect = 0;
unsigned long lastWiFiConnect = 0;


unsigned int countCheckInternetBad = 0;
bool heatingEnabled = true;
bool enableHotWater = false;
unsigned char boiler_Fault = 0;
float boiler_Modulation = 0;

//#define MSG_BUFFER_SIZE  (50)
//char msg[MSG_BUFFER_SIZE];

bool needSave = false;


OpenTherm ot(OT_IN_PIN, OT_OUT_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

void IRAM_ATTR handleInterrupt() {
  ot.handleInterrupt();
}

String Topic (const char* st )
{
  String st1 = TOPIC; 
  String st2 = st; 
  return st1 + st2;
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

  if (responseStatus == OpenThermResponseStatus::SUCCESS) {
    boiler.op = boiler.Pid();
    ot.setBoilerTemperature(boiler.op);
  }
  
  unsigned long tnow = millis();
  if (ot.isHotWaterActive(response))
  {
    ot.setDHWSetpoint(t_hot_water);
    Serial.print("Set temperature hot water: " + String(t_hot_water) + " °C ");
  } 


  Serial.println("Current temperature: " + String(boiler.t) + " °C ");
  String tempSource = boiler.workTempIsExternal
                      ? "(external sensor)"
                      : "(internal sensor)";
  Serial.println(tempSource);

  if (!client.connected()) return;

  float bt = ot.getBoilerTemperature();
  float wt = ot.getDHWTemperature();
  
  client.publish(Topic(TEMP_BOILER_TARGET_GET_TOPIC).c_str(), String(boiler.op).c_str());
  client.publish(Topic(CURRENT_TEMP_GET_TOPIC).c_str(), String(boiler.t).c_str());
  client.publish(Topic(TEMP_BOILER_GET_TOPIC).c_str(), String(bt).c_str());
  client.publish(Topic(TEMP_SETPOINT_GET_TOPIC).c_str(), String(boiler.sp).c_str());
  client.publish(Topic(MODE_GET_TOPIC).c_str(), heatingEnabled ? "heat" : "off");
  client.publish(Topic(MODE_HOT_WATER_GET_TOPIC).c_str(), enableHotWater ? "heat" : "off");
  client.publish(Topic(TEMP_HOT_WATER_GET_TOPIC).c_str(), String(wt).c_str());
  client.publish(Topic(TEMP_HOT_WATER_GET_CUR_TOPIC).c_str(), String(t_hot_water).c_str());
  client.publish(Topic(BOILER_MODULATION_TOPIC).c_str(), String(boiler_Modulation).c_str());
  client.publish(Topic(INTERNAL_TEMP_CORRECT_GET_TOPIC).c_str(), String(boiler.internalTempCorrect).c_str());
  client.publish(Topic(MAX_BOILER_TEMP_GET_TOPIC).c_str(), String(boiler.ophi).c_str());
  client.publish(Topic(INT_SENSOR_TEMP_GET_TOPIC).c_str(), String(boiler.internalTemp).c_str());
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
    if ((boiler.sp > SPMAX) || (boiler.sp < SPMIN)) Serial.print("Target temperature not change! ");
    else boiler.sp = payloadStr.toFloat();
    Serial.println("Set target temperature: " + payloadStr);
    needSave = true;
  }
  else if (topicStr == currentTempSetTopic) {
    boiler.SetExtTemp(payloadStr.toFloat());
  }
  else if (topicStr == setCorrectInternalTempTopic) {
    boiler.internalTempCorrect = payloadStr.toFloat();
    Serial.println("Set correct: " + String(boiler.internalTempCorrect));
    needSave = true;
  }
  else if (topicStr == tempSetHotWaterTopic) {
    t_hot_water = payloadStr.toFloat();
    Serial.println("Hot water temperature: " + String(t_hot_water));
    needSave = true;
  }
  else if (topicStr == maxBoilerTempTopic) {
    boiler.ophi = payloadStr.toFloat();
    if ((boiler.ophi > OPHIMAX) || (boiler.ophi < OPLO)) {
      Serial.print("Maximum boiler not change! ");
    }
    Serial.println("Temperature: " + String(boiler.ophi));
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
  
  boiler.CheckConfig();

  mwifi.SetConfig(myserv.getConfig().SSID,myserv.getConfig().PSW);
  mwifi.SetHostname("Boiler");
  mwifi.AutoWiFi();

  myserv.startServer();

  ArduinoOTA.setHostname("OpenTherm");
  ArduinoOTA.setPassword((const char *)"3422");
  ArduinoOTA.begin();   // You should set a password for OTA. Ideally using MD5 hashes
  
  boiler.Init();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  ot.begin(handleInterrupt);
}


void ReconnectMqtt() {
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
  boiler.loop();
  unsigned long now = millis();
  if (mwifi.status == SoftAP || mwifi.status == Connected)
    ArduinoOTA.handle();

  // проверка интернета провалилась более ...  
  if (countCheckInternetBad>5)
  {
    if (strlen(myserv.getConfig().SSID) > 0) mwifi.ReconnectWiFi();
    countCheckInternetBad = 0;
  }

  // В режиме точки доступа пробуем подключится к WiFi
  if (mwifi.status == SoftAP) {
    if (now - lastWiFiConnect > statusConnectWiFi_ms) {
      if (strlen(myserv.getConfig().SSID) > 0) mwifi.AutoWiFi();
      lastWiFiConnect = now;
    }
  }
  
  // Проверяем интернет в случае если есть обрыв связи с сервером, 
  // если есть подключение к wifi и есть интернет  подключаемся повторно
  if (!client.connected()) 
  {
    if (now - lastMQTTConnect > statusConnectMQTT_ms) 
    {
      if (mwifi.status == Connected && mwifi.WifiIsConnect())
      {
        if (mwifi.InternetIsConnect()) ReconnectMqtt();
      }
      else countCheckInternetBad++;  
      lastMQTTConnect = now;
    }
  }
  else 
  {
    countCheckInternetBad = 0;
    client.loop();
  }
  

  // сохраняем конфиг при необходимости
  if (needSave) {
    myserv.saveConfig(PATH_SETJSON);
    Serial.println(" Save config!");
    needSave = false;
  }

  // основная работа по отправке значений и корректировки температур
  if (now - lastUpdate > statusUpdateInterval_ms) {
    lastUpdate = now;
    myserv.handleClientServer();
    updateData();
  }
}