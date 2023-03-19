// Your WiFi credentials.
// Set password to "" for open networks.
//const char* ssid = "MiTS";
//const char* pass = "Anton-3422";
// Set password to "" for open networks.
const char* ssid = "ZosiWiFi";
const char* pass = "Zosi-3422";
const char* ssid2 = "ZosiWiFi";
const char* pass2 = "Zosi-3422";
// Your MQTT broker address and credentials
const char* mqtt_server = "194.146.38.199";
const char* mqtt_user = "anto4a";
const char* mqtt_password = "Mosq3422M";
const int mqtt_port = 1883;

// Master OpenTherm Shield pins configuration
const int OT_OUT_PIN = D1; //for Arduino, 5 for ESP8266 (D1), 22 for ESP32
const int OT_IN_PIN = D2;  //for Arduino, 4 for ESP8266 (D2), 21 for ESP32

#define OPLO      20
#define OPHIDEF   65
#define OPHIMAX   85
#define SPDEF     20
#define SPMIN     5
#define SPMAX     30

// Temperature sensor pin
const int ROOM_TEMP_SENSOR_PIN = D3; //for Arduino, 14 for ESP8266 (D5), 18 for ESP32

const char* TOPIC = "opentherm-thermostat_";
// MQTT topics
const char* CURRENT_TEMP_GET_TOPIC = "/current-temperature/get";
const char* CURRENT_TEMP_SET_TOPIC = "/current-temperature/set";

const char* TEMP_SETPOINT_GET_TOPIC = "/setpoint-temperature/get";
const char* TEMP_SETPOINT_SET_TOPIC = "/setpoint-temperature/set";

const char* MODE_GET_TOPIC = "/mode/get";
const char* MODE_SET_TOPIC = "/mode/set";

const char* MODE_HOT_WATER_GET_TOPIC = "/mode-hot-water/get";
const char* MODE_HOT_WATER_SET_TOPIC = "/mode-hot-water/set";

const char* TEMP_HOT_WATER_GET_TOPIC = "/hot-water-temperature/get";
const char* TEMP_HOT_WATER_SET_TOPIC = "/hot-water-temperature/set";
const char* TEMP_HOT_WATER_GET_CUR_TOPIC = "/hot-water-current-temperature/get";

const char* TEMP_BOILER_GET_TOPIC = "/boiler-temperature/get";
const char* TEMP_BOILER_TARGET_GET_TOPIC = "/boiler-target-temperature/get";

const char* BOILER_FAULT_TOPIC = "/boiler-fault/get";
const char* BOILER_MODULATION_TOPIC = "/boiler-modulation/get";

const char* INTERNAL_TEMP_CORRECT_GET_TOPIC = "/int-correct-temp/get";
const char* INTERNAL_TEMP_CORRECT_SET_TOPIC = "/int-correct-temp/set";

const char* MAX_BOILER_TEMP_GET_TOPIC = "/max-boiler-temp/get";
const char* MAX_BOILER_TEMP_SET_TOPIC = "/max-boiler-temp/set";

const char* INT_SENSOR_TEMP_GET_TOPIC = "/int-sensor-temperature/get";

/*#define PATH_SETJSON          "/devset.json"

struct Config {
  char hostname[32];
  char SSID[SSID_LEN];
  char PSW[PSW_LEN];
  uint32_t device_ip;
  uint32_t gateway_ip;
  uint32_t subnet_mask;
  uint32_t dns_ip;
  uint8_t dhcp;
  uint16_t alarmDat;
  uint16_t modulePeriodSleep;
  uint8_t modulePeriodDat;
  uint8_t modulePeriodTransmit;
  uint16_t moduleAlarmDat;
  uint8_t moduleLoraadr;
  uint8_t moduleNamberDat;
  uint8_t mainReload;
  uint8_t telegramSend;
}; */