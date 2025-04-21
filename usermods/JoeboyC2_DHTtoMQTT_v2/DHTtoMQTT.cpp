#include "wled.h"

#ifdef WLED_DISABLE_MQTT
#error "This user mod requires MQTT to be enabled."
#endif

#include <dht_nonblocking.h>

// DHT Configuration
#ifndef USERMOD_DHT_DHTTYPE
#define USERMOD_DHT_DHTTYPE 22
#endif

#if DHT_TYPE == 11
#define DHTTYPE DHT_TYPE_11
#elif DHT_TYPE == 21
#define DHTTYPE DHT_TYPE_21
#elif DHT_TYPE == 22
#define DHTTYPE DHT_TYPE_22
#endif

// Connect pin 1 (on the left) of the sensor to +5V
//   NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
//   to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
//   NOTE: Pin defaults below are for QuinLed Dig-Uno's Q2 on the board
// Connect pin 4 (on the right) of the sensor to GROUND
//   NOTE: If using a bare sensor (AM*), Connect a 10K resistor from pin 2
//   (data) to pin 1 (power) of the sensor. DHT* boards have the pullup already

#ifdef DHT_PIN
#define DHTPIN DHT_PIN
#else
#ifdef ARDUINO_ARCH_ESP32
#define DHTPIN 21
#else //ESP8266 boards
#define DHTPIN 4
#endif
#endif

// the frequency to check sensor, 1 minute
#ifndef DHT_MEASUREMENT_INTERVAL
#define DHT_MEASUREMENT_INTERVAL 60000
#endif

// how many seconds after boot to take first measurement, 90 seconds
// 90 gives enough time to OTA update firmware if this crashes
#ifndef DHT_FIRST_MEASUREMENT_AT
#define DHT_FIRST_MEASUREMENT_AT 90000
#endif

#define DHT_TIMEOUT_TIME 10000

class UsermodDHTtoMQTT : public Usermod {
  private:
    DHT_nonblocking* dht_sensor = nullptr;
    unsigned long nextReadTime = 0;
    unsigned long lastReadTime = 0;
    float humidity = 0;
    float temperature = 0;
    bool initializing = true;
    bool disabled = false;
    char mqttTempTopic[64];
    char mqttHumidTopic[64];
    bool mqttInitialized = false;

    // Config variables
    uint8_t dhtPin = DHTPIN;
    uint8_t dhtType = DHTTYPE;
    bool enabled = true;
    bool useCelsius = true;
    uint32_t measurementInterval = DHT_MEASUREMENT_INTERVAL;
    bool initDone = false;

    unsigned long lastSuccessfulRead = 0;
    const unsigned long ERROR_TIMEOUT = 10000; // 10 seconds timeout for readings
    bool sensorError = false;
    bool hadSuccessfulRead = false;  // Track if we've ever had a successful reading

  public:
    void setup() {
      if (dht_sensor) delete dht_sensor;
      dht_sensor = new DHT_nonblocking(dhtPin, dhtType);
      nextReadTime = millis() + DHT_FIRST_MEASUREMENT_AT;
      lastReadTime = millis();

      // Initialize MQTT topics
      snprintf(mqttTempTopic, sizeof(mqttTempTopic), "%s/temperature", mqttDeviceTopic);
      snprintf(mqttHumidTopic, sizeof(mqttHumidTopic), "%s/humidity", mqttDeviceTopic);
    }

    void loop() {
      if (disabled || !enabled) return;
      if (millis() < nextReadTime) return;

      float tempC;
      if (dht_sensor->measure(&tempC, &humidity)) {
        temperature = useCelsius ? tempC : tempC * 9 / 5 + 32;
        lastSuccessfulRead = millis();
        sensorError = false;
        hadSuccessfulRead = true;

        // Handle MQTT if connected
        if (WLED_MQTT_CONNECTED) {
          if (!mqttInitialized) {
            publishMqttDeviceConfig();
            mqttInitialized = true;
          }
          publishSensorData();
        } else {
          mqttInitialized = false;
        }

        nextReadTime = millis() + measurementInterval;
        lastReadTime = millis();
        initializing = false;
      } else {
        // Check if we haven't had a successful reading for too long
        if (millis() - lastSuccessfulRead > ERROR_TIMEOUT) {
          sensorError = true;
          if (WLED_MQTT_CONNECTED && hadSuccessfulRead) {
            publishSensorData(); // Publish error state to MQTT
          }
          DEBUG_PRINTLN(F("DHT sensor read error!"));
        }
      }

      if ((millis() - lastSuccessfulRead) > 10 * measurementInterval) {
        disabled = true;
        DEBUG_PRINTLN(F("DHT sensor disabled due to repeated failures"));
      }
    }

    void publishMqttDeviceConfig() {
      // Temperature sensor config
      publishSensorConfig("temperature", mqttTempTopic, "temperature", 
                         useCelsius ? "°C" : "°F");
      // Humidity sensor config
      publishSensorConfig("humidity", mqttHumidTopic, "humidity", "%");
    }

    void publishSensorConfig(const char* name, const char* topic, 
                           const char* device_class, const char* unit) {
      char configTopic[128];
      snprintf(configTopic, sizeof(configTopic), 
               "homeassistant/sensor/%s/%s/config", mqttClientID, name);

      StaticJsonDocument<300> doc;
      doc["name"] = name;
      doc["state_topic"] = topic;
      doc["unique_id"] = String(mqttClientID) + name;
      doc["device_class"] = device_class;
      doc["unit_of_measurement"] = unit;
      doc["expire_after"] = 1800;

      JsonObject device = doc.createNestedObject("device");
      device["identifiers"] = String("wled-") + mqttClientID;
      device["manufacturer"] = "Aircoookie";
      device["model"] = "WLED";
      device["sw_version"] = VERSION;
      device["name"] = mqttClientID;

      String configJson;
      serializeJson(doc, configJson);
      mqtt->publish(configTopic, 0, true, configJson.c_str());
    }

    void publishSensorData() {
      if (sensorError) {
        mqtt->publish(mqttTempTopic, 0, true, "Sensor Error");
        mqtt->publish(mqttHumidTopic, 0, true, "Sensor Error");
        return;
      }

      char tempStr[10], humStr[10];
      dtostrf(temperature, 1, 2, tempStr);
      dtostrf(humidity, 1, 2, humStr);
      
      mqtt->publish(mqttTempTopic, 0, true, tempStr);
      mqtt->publish(mqttHumidTopic, 0, true, humStr);
    }

    void addToJsonInfo(JsonObject& root) {
      if (disabled || !enabled) return;
      
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray temp = user.createNestedArray("Temperature");
      JsonArray hum = user.createNestedArray("Humidity");

      if (initializing) {
        temp.add("Initializing ");
        temp.add((nextReadTime - millis()) / 1000);
        hum.add("Initializing ");
        hum.add((nextReadTime - millis()) / 1000);
        return;
      }

      if (sensorError) {
        temp.add("Error - ");
        temp.add("Sensor not responding");
        hum.add("Error - ");
        hum.add("Sensor not responding");
        return;
      }

      // Only show readings if we've had at least one successful read
      if (hadSuccessfulRead) {
        temp.add(temperature);
        temp.add(useCelsius ? "°C" : "°F");
        
        hum.add(humidity);
        hum.add("%");
      } else {
        temp.add("Waiting");
        temp.add("No reading yet");
        hum.add("Waiting");
        hum.add("No reading yet");
      }
    }

    void appendConfigData() {
      oappend(SET_F("addInfo('DHT-pin',{type:'number',caption:'DHT Pin',min:0,max:40,val:"));
      oappend(String(dhtPin).c_str());
      oappend(SET_F("});"));

      oappend(SET_F("addInfo('DHT-type',{type:'select',caption:'DHT Type',options:[[11,'DHT11'],[21,'DHT21'],[22,'DHT22']],val:"));
      oappend(String(dhtType).c_str());
      oappend(SET_F("});"));

      oappend(SET_F("addInfo('DHT-en',{type:'bool',caption:'Enabled',val:"));
      oappend(enabled ? "true" : "false");
      oappend(SET_F("});"));

      oappend(SET_F("addInfo('DHT-unit',{type:'bool',caption:'Use Celsius',val:"));
      oappend(useCelsius ? "true" : "false");
      oappend(SET_F("});"));

      oappend(SET_F("addInfo('DHT-interval',{type:'number',caption:'Measurement Interval (ms)',min:10000,max:300000,val:"));
      oappend(String(measurementInterval).c_str());
      oappend(SET_F("});"));
    }

    bool readFromConfig(JsonObject& root) {
      JsonObject top = root[FPSTR("DHT")];
      
      if (top.isNull()) {
        DEBUG_PRINTLN(F("No config found. Creating."));
        return false;
      }

      bool configComplete = !top[FPSTR("pin")].isNull();

      if (!initDone) {
        dhtPin = top[FPSTR("pin")] | dhtPin;
        dhtType = top[FPSTR("type")] | dhtType;
        enabled = top[FPSTR("enabled")] | enabled;
        useCelsius = top[FPSTR("celsius")] | useCelsius;
        measurementInterval = top[FPSTR("interval")] | measurementInterval;
        
        if (dht_sensor) delete dht_sensor;
        dht_sensor = new DHT_nonblocking(dhtPin, dhtType);
        
        initDone = true;
      } else {
        if (!top[FPSTR("pin")].isNull()) {
          dhtPin = top[FPSTR("pin")] | dhtPin;
          if (dht_sensor) delete dht_sensor;
          dht_sensor = new DHT_nonblocking(dhtPin, dhtType);
        }
        if (!top[FPSTR("type")].isNull()) {
          dhtType = top[FPSTR("type")] | dhtType;
          if (dht_sensor) delete dht_sensor;
          dht_sensor = new DHT_nonblocking(dhtPin, dhtType);
        }
        if (!top[FPSTR("enabled")].isNull()) enabled = top[FPSTR("enabled")] | enabled;
        if (!top[FPSTR("celsius")].isNull()) useCelsius = top[FPSTR("celsius")] | useCelsius;
        if (!top[FPSTR("interval")].isNull()) measurementInterval = top[FPSTR("interval")] | measurementInterval;
      }
      
      return configComplete;
    }

    void addToConfig(JsonObject& root) {
      JsonObject top = root.createNestedObject(F("DHT"));
      top[FPSTR("pin")] = dhtPin;
      top[FPSTR("type")] = dhtType;
      top[FPSTR("enabled")] = enabled;
      top[FPSTR("celsius")] = useCelsius;
      top[FPSTR("interval")] = measurementInterval;
    }

    uint16_t getId() {
      return USERMOD_ID_DHT;
    }
};

static UsermodDHTtoMQTT dhtMqtt;
REGISTER_USERMOD(dhtMqtt);