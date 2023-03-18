
#ifndef myserver_h
#define myserver_h

#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include "LittleFS.h"

// названия файлов
#define PATH_LIST_ID          "/idList.txt"
#define PATH_INDEX_HTML       "/index.html"
#define PATH_T_ZONE_HTML      "/timezone.html"
#define PATH_INDEX_SCRIPT     "/indexscript.html"
#define PATH_SETDEV_SCRIPT    "/settingscript.html"
#define PATH_SET_DEV_HTML     "/setting_device.html"
#define PATH_SET_WIFI_HTML    "/setting_wifi.html"
#define PATH_RESTART_HTML     "/restart.html"
#define PATH_SAVE_HTML        "/save.html"
#define PATH_HAED_HTML        "/head.html"
#define PATH_SETDEV           "/devset.txt"
#define PATH_SETJSON          "/devset.json"
#define PATH_HAEDUPD_HTML     "/headupd.html"

#define SSID_JS_STR           F("ssid")
#define PSW_JS_STR            F("password")

//#define HOSTNAME  "Clock&Timer" 

#define SSID_LEN  32
#define PSW_LEN  32

struct Config {
  char hostname[32];
  char SSID[SSID_LEN];
  char PSW[PSW_LEN];
  int timeZone;
}; 

class MyServer
{
  public:
    Config getConfig();
    void startServer();
    void saveConfig(const char *filename);
    bool loadConfig(const char *filename);
    void handleClientServer ();
  //private:
};

#endif