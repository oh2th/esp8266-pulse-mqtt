#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <EEPROM.h>
#include <math.h>

#define PULSE_PIN D6
#define DEBOUNCE_MS 50
#define CONFIG_FILE "/config.json"
#define EEPROM_SIZE 8
#define EEPROM_ADDR 0

// MQTT topic and reporting interval
#define REPORT_INTERVAL 60  // seconds

struct Config {
  char wifi_ssid[32];
  char wifi_pass[32];
  char mqtt_host[32];
  uint16_t mqtt_port;
  float meter_water;
  uint16_t litres_per_pulse;
  char mqtt_topic[64];
} config;

volatile unsigned long lastPulse = 0;
volatile uint32_t pulseCount = 0;

AsyncWebServer server(80);
AsyncMqttClient mqttClient;
Ticker debounceTicker;

unsigned long lastReport = 0;
unsigned long bootTime = 0;
uint32_t lastPulseCount = 0;

// Load configuration from LittleFS
bool loadConfig() {
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return false;
  }
  if (!LittleFS.exists(CONFIG_FILE)) {
    Serial.println("Config file not found. Initial setup required.");
    return false;
  }
  File file = LittleFS.open(CONFIG_FILE, "r");
  if (!file) {
    Serial.println("Failed to open config file");
    return false;
  }
  StaticJsonDocument<384> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) {
    Serial.println("Failed to parse config file");
    return false;
  }
  strlcpy(config.wifi_ssid, doc["wifi_ssid"] | "", sizeof(config.wifi_ssid));
  strlcpy(config.wifi_pass, doc["wifi_pass"] | "", sizeof(config.wifi_pass));
  strlcpy(config.mqtt_host, doc["mqtt_host"] | "", sizeof(config.mqtt_host));
  config.mqtt_port = doc["mqtt_port"] | 1883;
  config.meter_water = doc["meter_water"] | 0.0;
  config.litres_per_pulse = doc["litres_per_pulse"] | 10;
  strlcpy(config.mqtt_topic, doc["mqtt_topic"] | "watermeter/data", sizeof(config.mqtt_topic));

  Serial.println("Loaded configuration:");
  Serial.printf("  WiFi SSID: %s\n", config.wifi_ssid);
  Serial.printf("  MQTT Host: %s\n", config.mqtt_host);
  Serial.printf("  MQTT Port: %u\n", config.mqtt_port);
  Serial.printf("  MQTT Topic: %s\n", config.mqtt_topic);
  Serial.printf("  Litres per Pulse: %u\n", config.litres_per_pulse);
  Serial.printf("  Meter Water: %.3f m3\n", config.meter_water);
  return true;
}

// Save configuration to LittleFS
bool saveConfig() {
  StaticJsonDocument<384> doc;
  doc["wifi_ssid"] = config.wifi_ssid;
  doc["wifi_pass"] = config.wifi_pass;
  doc["mqtt_host"] = config.mqtt_host;
  doc["mqtt_port"] = config.mqtt_port;
  doc["meter_water"] = config.meter_water;
  doc["litres_per_pulse"] = config.litres_per_pulse;
  doc["mqtt_topic"] = config.mqtt_topic;
  File file = LittleFS.open(CONFIG_FILE, "w");
  if (!file) {
    Serial.println("Failed to open config file for writing");
    return false;
  }
  serializeJson(doc, file);
  file.close();
  return true;
}

void IRAM_ATTR onPulse() {
  unsigned long now = millis();
  if (now - lastPulse > DEBOUNCE_MS) {
    pulseCount++;
    lastPulse = now;
  }
}

void setupWebServer() {
  // Serve config form (GET)
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><body><h2>Configuration</h2><form method='POST' action='/config'>";
    html += "WiFi SSID: <input name='wifi_ssid' value='" + String(config.wifi_ssid) + "'><br>";
    html += "WiFi Password: <input name='wifi_pass' type='password' value='" + String(config.wifi_pass) + "'><br>";
    html += "MQTT Host: <input name='mqtt_host' value='" + String(config.mqtt_host) + "'><br>";
    html += "MQTT Port: <input name='mqtt_port' type='number' value='" + String(config.mqtt_port) + "'><br>";
    html += "MQTT Topic Prefix: <input name='mqtt_topic' value='" + String(config.mqtt_topic) + "'><br>";
    html += "<small>Final topic will be: <b>" + String(config.mqtt_topic) + "/state</b></small><br>";
    html += "Litres per Pulse: <input name='litres_per_pulse' type='number' value='" + String(config.litres_per_pulse) + "'><br>";
    html += "Meter Water (m3): <input name='meter_water' value='" + String(config.meter_water, 3) + "'><br>";
    html += "<input type='submit' value='Save'></form></body></html>";
    request->send(200, "text/html", html);
  });

  // Handle config form submission (POST)
  server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request) {
    bool changed = false;
    if (request->hasParam("wifi_ssid", true)) {
      strlcpy(config.wifi_ssid, request->getParam("wifi_ssid", true)->value().c_str(), sizeof(config.wifi_ssid));
      changed = true;
    }
    if (request->hasParam("wifi_pass", true)) {
      strlcpy(config.wifi_pass, request->getParam("wifi_pass", true)->value().c_str(), sizeof(config.wifi_pass));
      changed = true;
    }
    if (request->hasParam("mqtt_host", true)) {
      strlcpy(config.mqtt_host, request->getParam("mqtt_host", true)->value().c_str(), sizeof(config.mqtt_host));
      changed = true;
    }
    if (request->hasParam("mqtt_port", true)) {
      config.mqtt_port = request->getParam("mqtt_port", true)->value().toInt();
      changed = true;
    }
    if (request->hasParam("mqtt_topic", true)) {
      strlcpy(config.mqtt_topic, request->getParam("mqtt_topic", true)->value().c_str(), sizeof(config.mqtt_topic));
      changed = true;
    }
    if (request->hasParam("litres_per_pulse", true)) {
      config.litres_per_pulse = request->getParam("litres_per_pulse", true)->value().toInt();
      changed = true;
    }
    if (request->hasParam("meter_water", true)) {
      config.meter_water = request->getParam("meter_water", true)->value().toFloat();
      changed = true;
    }
    if (changed) {
      saveConfig();
      request->send(200, "text/html", "<html><body><h2>Configuration saved. Rebooting...</h2></body></html>");
      delay(1000);
      ESP.restart();
    } else {
      request->redirect("/config");
    }
  });

  // Optionally, serve a root page or redirect to /config
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/config");
  });

  server.begin();
}

// WiFi connect logic
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifi_ssid, config.wifi_pass);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
}

// MQTT connect logic
void connectMQTT() {
  mqttClient.setServer(config.mqtt_host, config.mqtt_port);
  mqttClient.connect();
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Build the expected topic: <prefix>/set
  char setTopic[80];
  snprintf(setTopic, sizeof(setTopic), "%s/set", config.mqtt_topic);

  if (strcmp(topic, setTopic) == 0) {
    // Parse payload as JSON or plain float
    payload[len] = '\0'; // Ensure null-terminated
    float newMeter = NAN;

    // Try to parse as JSON: {"meter_water": 12.345}
    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, payload, len);
    if (!err && doc.containsKey("meter_water")) {
      newMeter = doc["meter_water"];
    } else {
      // Try to parse as plain float
      newMeter = atof(payload);
    }

    if (!isnan(newMeter) && newMeter >= 0.0 && newMeter < 100000.0) {
      Serial.printf("MQTT: Setting meter_water to %.3f m3 via MQTT\n", newMeter);
      config.meter_water = newMeter;
      saveMeterWaterToEEPROM(config.meter_water);
      // Optionally, saveConfig(); // Only if you want to persist to FS immediately
    } else {
      Serial.println("MQTT: Invalid meter_water value received");
    }
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT broker.");

  // Subscribe to <prefix>/set
  char setTopic[80];
  snprintf(setTopic, sizeof(setTopic), "%s/set", config.mqtt_topic);
  mqttClient.subscribe(setTopic, 0);
  Serial.printf("Subscribed to MQTT topic: %s\n", setTopic);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT, reconnecting...");
  delay(1000);
  connectMQTT();
}

// Publish water meter data to MQTT
void publishMQTT(float measure_water, float meter_water, unsigned long uptime) {
  StaticJsonDocument<128> doc;
  doc["uptime"] = uptime;
  doc["measure_water"] = measure_water;
  doc["meter_water"] = meter_water;
  char payload[128];
  size_t n = serializeJson(doc, payload, sizeof(payload));

  // Build topic: <prefix>/state
  char topic[80];
  snprintf(topic, sizeof(topic), "%s/state", config.mqtt_topic);

  mqttClient.publish(topic, 0, false, payload, n);
}

// Captive portal logic: if not connected to WiFi, start AP and serve /config
void startCaptivePortal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("PulseMeterConfig");
  Serial.println("Started AP: PulseMeterConfig");
  // Web server already serves /config
}

// Save meter_water to EEPROM
void saveMeterWaterToEEPROM(float value) {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_ADDR, value);
  EEPROM.commit();
  EEPROM.end();
}

// Load meter_water from EEPROM
float loadMeterWaterFromEEPROM() {
  float value = 0.0;
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDR, value);
  EEPROM.end();
  return value;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- Pulse Meter Startup ---");

  LittleFS.begin();
  bool configLoaded = loadConfig();

  // Restore meter_water from EEPROM (overrides config file value)
  float eepromValue = loadMeterWaterFromEEPROM();
  if (isnan(eepromValue) || eepromValue < 0.0 || eepromValue > 100000.0) {
    Serial.println("EEPROM meter_water invalid or not set.");
    config.meter_water = config.meter_water > 0.0 ? config.meter_water : 0.0;
  } else {
    Serial.printf("Restored meter_water from EEPROM: %.3f m3\n", eepromValue);
    config.meter_water = eepromValue;
  }

  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed. Starting captive portal.");
    startCaptivePortal();
  } else {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  }

  // MQTT setup
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  connectMQTT();

  setupWebServer();
  pinMode(PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), onPulse, FALLING);

  bootTime = millis();
}

void loop() {
  unsigned long now = millis();

  static unsigned long nextReport = 0;
  static float lastSavedMeterWater = 0.0;
  static unsigned long lastConfigSave = 0;

  if (nextReport == 0) {
    nextReport = now + REPORT_INTERVAL * 1000;
    lastConfigSave = now;
  }

  // Calculate measure_water (l/min) every REPORT_INTERVAL seconds, adjusting for drift
  if ((long)(now - nextReport) >= 0) {
    uint32_t pulses = pulseCount;
    uint32_t deltaPulses = pulses - lastPulseCount;
    lastPulseCount = pulses;

    float litres = deltaPulses * config.litres_per_pulse;
    float measure_water = (litres / (float)REPORT_INTERVAL); // l/min
    config.meter_water += litres / 1000.0; // m3

    unsigned long uptime = (now - bootTime) / 1000;

    // Prepare JSON for MQTT and logging
    StaticJsonDocument<128> doc;
    doc["uptime"] = uptime;
    doc["measure_water"] = measure_water;
    doc["meter_water"] = config.meter_water;
    char payload[128];
    size_t n = serializeJson(doc, payload, sizeof(payload));

    // Output JSON to serial
    Serial.print("Publishing: ");
    Serial.println(payload);

    // Publish to MQTT
    mqttClient.publish(config.mqtt_topic, 0, false, payload, n);

    // Save to EEPROM if changed by at least 0.01 m3
    if (fabs(config.meter_water - lastSavedMeterWater) >= 0.01) {
      saveMeterWaterToEEPROM(config.meter_water);
      lastSavedMeterWater = config.meter_water;
    }

    // Save full config to LittleFS every 24 hours (86,400,000 ms)
    if (now - lastConfigSave >= 86400000UL) {
      saveConfig();
      lastConfigSave = now;
    }

    // Schedule next report, preserving interval accuracy
    nextReport += REPORT_INTERVAL * 1000;
    if ((long)(now - nextReport) >= 0) {
      nextReport = now + REPORT_INTERVAL * 1000;
    }
  }

  // If WiFi disconnected, start captive portal for config
  if (WiFi.status() != WL_CONNECTED) {
    startCaptivePortal();
  }
}
