/*************************************************************
  This example runs directly on ESP8266 chip.

  Please be sure to select the right ESP8266 module
  in the Tools -> Board -> WeMos D1 Mini

  Adjust settings in Config.h before run
 *************************************************************/

#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <OpenTherm.h>

// Your WiFi credentials.
// Set password to "" for open networks.
const char* ssid2 = "MiTS";
const char* pass2 = "Anton-3422";
// Set password to "" for open networks.
const char* ssid = "ZosiWiFi";
const char* pass = "Zosi-3422";
// Your MQTT broker address and credentials
const char* mqtt_server = "194.146.38.199";
const char* mqtt_user = "anto4a";
const char* mqtt_password = "Mosq3422M";
const int mqtt_port = 1883;

// Master OpenTherm Shield pins configuration
const int OT_OUT_PIN = D1; //for Arduino, 5 for ESP8266 (D1), 22 for ESP32
const int OT_IN_PIN = D2;  //for Arduino, 4 for ESP8266 (D2), 21 for ESP32

// Temperature sensor pin
const int ROOM_TEMP_SENSOR_PIN = D3; //for Arduino, 14 for ESP8266 (D5), 18 for ESP32

// MQTT topics
const char* CURRENT_TEMP_GET_TOPIC = "opentherm-thermostat/current-temperature/get";
const char* CURRENT_TEMP_SET_TOPIC = "opentherm-thermostat/current-temperature/set";

const char* TEMP_SETPOINT_GET_TOPIC = "opentherm-thermostat/setpoint-temperature/get";
const char* TEMP_SETPOINT_SET_TOPIC = "opentherm-thermostat/setpoint-temperature/set";

const char* MODE_GET_TOPIC = "opentherm-thermostat/mode/get";
const char* MODE_SET_TOPIC = "opentherm-thermostat/mode/set";

const char* MODE_HOT_WATER_GET_TOPIC = "opentherm-thermostat/mode-hot-water/get";
const char* MODE_HOT_WATER_SET_TOPIC = "opentherm-thermostat/mode-hot-water/set";

const char* TEMP_HOT_WATER_GET_TOPIC = "opentherm-thermostat/hot-water-temperature/get";
const char* TEMP_HOT_WATER_SET_TOPIC = "opentherm-thermostat/hot-water-temperature/set";

const char* TEMP_BOILER_GET_TOPIC = "opentherm-thermostat/boiler-temperature/get";
const char* TEMP_BOILER_TARGET_GET_TOPIC = "opentherm-thermostat/boiler-target-temperature/get";

const unsigned long intTempTimeout_ms = 10 * 1000;
const unsigned long extTempTimeout_ms = 60 * 1000;
const unsigned long statusUpdateInterval_ms = 1000;

float  sp = 15, //set point
       t = 15, //current temperature
       t_last = 0, //prior temperature
       ierr = 25, //integral error
       dt = 0, //time between measurements
       op = 0; //PID controller output
unsigned long ts = 0, new_ts = 0; //timestamp
unsigned long lastUpdate = 0;
unsigned long lastTempSet = 0;
unsigned long lastIntTempSet = 0;

bool heatingEnabled = true;
bool enableHotWater = false;

#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];

OneWire oneWire(ROOM_TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
OpenTherm ot(OT_IN_PIN, OT_OUT_PIN);
WiFiClient espClient;
PubSubClient client(espClient);

void ICACHE_RAM_ATTR handleInterrupt() {
  ot.handleInterrupt();
}

float getTemp() {
  unsigned long now = millis();
  if (now - lastTempSet > extTempTimeout_ms)
    return sensors.getTempCByIndex(0);
  else
    return t;
}

float pid(float sp, float pv, float pv_last, float& ierr, float dt) {
  float KP = 10;
  float KI = 0.02;
  // upper and lower bounds on heater level
  float ophi = 63;
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
  
  snprintf (msg, MSG_BUFFER_SIZE, "%s", String(op).c_str());
  client.publish(TEMP_BOILER_TARGET_GET_TOPIC, msg);

  snprintf (msg, MSG_BUFFER_SIZE, "%s", String(t).c_str());
  client.publish(CURRENT_TEMP_GET_TOPIC, msg);

  float bt = ot.getBoilerTemperature();
  snprintf (msg, MSG_BUFFER_SIZE, "%s", String(bt).c_str());
  client.publish(TEMP_BOILER_GET_TOPIC, msg);

  snprintf (msg, MSG_BUFFER_SIZE, "%s", String(sp).c_str());
  client.publish(TEMP_SETPOINT_GET_TOPIC, msg);

  snprintf (msg, MSG_BUFFER_SIZE, "%s", heatingEnabled ? "heat" : "off");
  client.publish(MODE_GET_TOPIC, msg);

  snprintf (msg, MSG_BUFFER_SIZE, "%s", enableHotWater ? "on" : "off");
  client.publish(MODE_HOT_WATER_GET_TOPIC, msg);

  Serial.print("Current temperature: " + String(t) + " °C ");
  String tempSource = (millis() - lastTempSet > extTempTimeout_ms)
                      ? "(internal sensor)"
                      : "(external sensor)";
  Serial.println(tempSource);
}

String convertPayloadToStr(byte* payload, unsigned int length) {
  char s[length + 1];
  s[length] = 0;
  for (int i = 0; i < length; ++i)
    s[i] = payload[i];
  String tempRequestStr(s);
  return tempRequestStr;
}

const String setpointSetTopic(TEMP_SETPOINT_SET_TOPIC);
const String currentTempSetTopic(CURRENT_TEMP_SET_TOPIC);
const String modeSetTopic(MODE_SET_TOPIC);
const String modeSetHotWaterTopic(MODE_HOT_WATER_SET_TOPIC);

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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    const char* clientId = "opentherm-thermostat-test";
    if (client.connect(clientId, mqtt_user, mqtt_password)) {
      Serial.println("ok");

      client.subscribe(TEMP_SETPOINT_SET_TOPIC);
      client.subscribe(MODE_SET_TOPIC);
      client.subscribe(CURRENT_TEMP_SET_TOPIC);
      client.subscribe(MODE_HOT_WATER_SET_TOPIC);
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("");
  Serial.print("Connecting to " + String(ssid));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  int deadCounter = 20;
  while (WiFi.status() != WL_CONNECTED && deadCounter-- > 0) {
    delay(500);
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
    while (true);
  }
  else {
    Serial.println("ok");
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  ot.begin(handleInterrupt);

  //Init DS18B20 sensor
  sensors.begin();
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); //switch to async mode
  t, t_last = sensors.getTempCByIndex(0);
  ts = millis();
  lastTempSet = -extTempTimeout_ms;
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastUpdate > statusUpdateInterval_ms) {
    lastUpdate = now;
    updateData();
  }
}

//RelModLevel