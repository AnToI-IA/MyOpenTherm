#include "MyServer.h"
#include "Boiler.h"
#include <string.h>
#include <stdio.h> 

ESP8266WebServer server(80);
Config mconfig;      // глобальный конфиг устройства, сохраняется в FS
extern float  inttemp;
extern float  sp,
              ophi;
extern float internalTempCorrect;
extern float t_hot_water;
extern bool heatingEnabled;
extern bool enableHotWater;
extern Boiler boiler;

Config MyServer::getConfig()
{
  return mconfig;
}

bool loadConfiguration(const char *filename, Config &config) {
  // Open file for reading
  File file = LittleFS.open(filename,"r");

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  strlcpy(config.SSID, doc["SSID"] | "", sizeof(config.SSID));  
  strlcpy(config.PSW, doc["PSW"] | "", sizeof(config.PSW));  
  boiler.sp = doc["SetPointTempr"];
  boiler.ophi = doc[("MaxBoilerTempr")];
  config.timeZone = doc["TimeZone"] | 0;

  heatingEnabled = doc[("HeatingEnabled")];
  enableHotWater = doc[("EnableHotWater")];

  internalTempCorrect = doc[("InternalTempCorrect")];

  t_hot_water = doc[("TempHotWater")];
  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
  #ifdef TESTMBREG
    file = LittleFS.open(filename,"r");
    String str = file.readString();
    Serial.println(str);
    file.close();
  #endif
  return !error;
}

void saveConfiguration(const char *filename, const Config &config) {
  // Delete existing file, otherwise the configuration is appended to the file
  LittleFS.remove(filename);

  // Open file for writing
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<256> doc;

  // Set the values in the document
  doc[("hostname")] = config.hostname;
  doc[("SSID")] = config.SSID;
  doc[("PSW")] = config.PSW;
  doc[("TimeZone")] = config.timeZone;
  doc[("SetPointTempr")] = boiler.sp;
  doc[("MaxBoilerTempr")] = boiler.ophi;
  doc[("HeatingEnabled")] = heatingEnabled;
  doc[("EnableHotWater")] = enableHotWater;
  doc[("InternalTempCorrect")] = internalTempCorrect;
  doc[("TempHotWater")] = t_hot_water;
  
  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
  }
  // Close the file
  file.close();
}

void MyServer::saveConfig(const char *filename)
{
  saveConfiguration(filename, mconfig);
}

bool MyServer::loadConfig(const char *filename) 
{
  return loadConfiguration(filename, mconfig);
}

void strAddOnFile(const char *filename, String* strout)
{
  File fileread = LittleFS.open(filename, "r");
  if (!fileread) 
  {
    #ifdef TESTMBREG
      Serial.print(F("Ошибка открытия файла"));
      Serial.println(filename);
    #endif
  }
  else 
  {
    *strout += fileread.readString();
    //Serial.println(*strout);
  }
  fileread.close();
}


void handleRoot() 
{
  String strfile;
  strAddOnFile(PATH_HAED_HTML,&strfile);
  strAddOnFile(PATH_INDEX_HTML,&strfile);
  strfile.replace(F("{IntTempr}"), String(inttemp));
  strAddOnFile(PATH_INDEX_SCRIPT,&strfile);
  server.send(200, "text/html", strfile);
  strfile.clear();
}

void handleSettingWifi()
{
  if ((server.arg(SSID_JS_STR) != "") && (server.arg(PSW_JS_STR) != ""))
  {
    if ((server.arg(SSID_JS_STR).length()+1)<=SSID_LEN)
      strlcpy(mconfig.SSID, server.arg(SSID_JS_STR).c_str(), server.arg(SSID_JS_STR).length()+1); 
    if ((server.arg(PSW_JS_STR).length()+1)<=PSW_LEN)
      strlcpy(mconfig.PSW, server.arg(PSW_JS_STR).c_str(), (server.arg(PSW_JS_STR).length()+1)); 
    saveConfiguration(PATH_SETJSON,mconfig);
  }
  String strfile;
  strAddOnFile(PATH_HAED_HTML,&strfile);
  strAddOnFile(PATH_SET_WIFI_HTML,&strfile);
  server.send(200, "text/html", strfile);
  strfile.clear();
}

void handleRestart()
{
  String strfile;
  //strAddOnFile(PATH_HAED_HTML,&strfile);
  strAddOnFile(PATH_RESTART_HTML,&strfile);
  server.send(200, "text/html", strfile);
  delay(500);
  ESP.restart();
}

void handleSettingDev() 
{
  String strfile;
  if (server.arg(F("timezone_offset")) != "")
  {
    mconfig.timeZone = atoi(server.arg(F("timezone_offset")).c_str());
    saveConfiguration(PATH_SETJSON,mconfig);
  }
  strAddOnFile(PATH_HAED_HTML,&strfile);
  strAddOnFile(PATH_T_ZONE_HTML,&strfile);
  strAddOnFile(PATH_SETDEV_SCRIPT,&strfile);

  //mconfig.timeZone
  String repl = "\""+String(mconfig.timeZone)+"\"";
  strfile.replace(repl, String(repl+"selected=\"selected\""));
  server.send(200, "text/html", strfile);
  strfile.clear();
}

void MyServer::startServer ()
{
  server.begin();
  server.on("/", handleRoot);
  server.on("/setting_wifi",  handleSettingWifi);
  server.on("/reset",  handleRestart);
  server.on("/setting_device",  handleSettingDev);
}

void MyServer::handleClientServer ()
{
  server.handleClient();
}
