// Your MQTT broker address and credentials
const char* mqtt_server = "194.146.38.199";
const char* mqtt_user = "anto4a";
const char* mqtt_password = "Mosq3422M";
const int mqtt_port = 1883;

// Master OpenTherm Shield pins configuration
const int OT_OUT_PIN = D1; //for Arduino, 5 for ESP8266 (D1), 22 for ESP32
const int OT_IN_PIN = D2;  //for Arduino, 4 for ESP8266 (D2), 21 for ESP32

const char* TOPIC = "opentherm-thermostat";
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
