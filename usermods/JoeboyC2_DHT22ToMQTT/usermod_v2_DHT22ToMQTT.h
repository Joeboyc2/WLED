#pragma once

#include "wled.h"
#include "Arduino.h"
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include "DHT_U.h"

// Set DHT22 PIN
#ifndef DHTPIN
    #ifdef ARDUINO_ARCH_ESP32
        #define DHTPIN 18
    #else //ESP8266 boards
        #define DHTPIN 14
    #endif
#endif

#define DHTTYPE DHT22   // DHT 22
// DHT instance created at runtime so the pin can be configurable
DHT *dht = nullptr;
#define TEMP_CELSIUS // Comment out for Fahrenheit

// Set the delay between temperature readings
#define MEASUREMENT_INTERVAL 60000

class Usermod_DHT22ToMQTT : public Usermod {
    private:
        // Record the last time a measurement was taken
        unsigned long lastMeasurement = 0; // Initialize to 0, will be set in setup()
        bool mqttInitialized = false;
        float SensorTemperature = NAN; // Use NAN (not a number) to handle invalid states
        float SensorHumidity = NAN;
        String mqttTemperatureTopic = "";
        String mqttHumidityTopic = "";
    unsigned long currentTime = 0;
        unsigned long lastPrintTime = 0;
        const unsigned long printInterval = 60000; // Print every 60 seconds
        bool enabled = true; // Enable by default
    // Configurable pin for DHT
    uint8_t dhtPin = DHTPIN;
        // Retry behavior for DHT failures (conservative) - made configurable
    uint8_t DHT_RETRIES = 3;
    unsigned long DHT_RETRY_DELAY_MS = 1500; // 1.5s between retries

    // Measurement interval (ms) - made configurable
    unsigned long measurementInterval = MEASUREMENT_INTERVAL;

        // strings to reduce flash memory usage (used more than twice)
    static const char _name[];
    static const char _enabled[];
    static const char _measurement_interval[]; // legacy ms key (backwards compat)
    static const char _measurement_interval_s[]; // seconds (preferred)
    static const char _retries[];
    static const char _retry_delay[]; // legacy ms key
    static const char _retry_delay_s[]; // seconds (preferred)
    static const char _pin[];
    static const char _restart_note[];

    public:
        void setup() {
            // DHT Setup
            Serial.println("Starting DHT!");
            Serial.println("Initialising Temperature sensor.. ");
            // instantiate DHT with the configured pin
            if (dht != nullptr) {
                delete dht;
                dht = nullptr;
            }
            dht = new DHT(dhtPin, DHTTYPE);
            dht->begin();
            // Set lastMeasurement to now so the first measurement happens after MEASUREMENT_INTERVAL
            lastMeasurement = millis();  // respect sensor warm-up delay
        }

        void _mqttInitialize() {
            Serial.println(F("Setting up MQTT"));
            mqttTemperatureTopic = String(mqttDeviceTopic) + "/temperature";
            mqttHumidityTopic = String(mqttDeviceTopic) + "/humidity";
            #ifdef TEMP_CELSIUS
                _createMqttSensor(F("temperature"), mqttTemperatureTopic, F("temperature"), F("°C"));
            #else
                _createMqttSensor(F("temperature"), mqttTemperatureTopic, F("temperature"), F("°F"));
            #endif
            _createMqttSensor(F("humidity"), mqttHumidityTopic, F("humidity"), F("%"));
        }

        void _createMqttSensor(const String &name, const String &topic, const String &deviceClass, const String &unitOfMeasurement) {
            String t = String("homeassistant/sensor/") + mqttClientID + "/" + name + "/config";

            StaticJsonDocument<300> doc;

            doc["name"] = name;
            doc["state_topic"] = topic;
            doc["unique_id"] = String(mqttClientID) + name;
            if (!unitOfMeasurement.isEmpty())
                doc["unit_of_measurement"] = unitOfMeasurement;
            if (!deviceClass.isEmpty())
                doc["device_class"] = deviceClass;
            doc["expire_after"] = 1800;

            JsonObject device = doc.createNestedObject("device"); // attach the sensor to the same device
            device["identifiers"] = String("wled-sensor-") + mqttClientID;
            device["manufacturer"] = "Aircoookie";
            device["model"] = "WLED";
            device["sw_version"] = VERSION;
            device["name"] = mqttClientID;

            String configData;
            serializeJson(doc, configData);
            Serial.println(t);
            Serial.println(configData);

            mqtt->publish(t.c_str(), 0, false, configData.c_str());
        }

        // Return true on successful read, false on failure
        bool _updateSensorData() {
            // Read temperature and humidity (DHT can take 250ms - 2s to read)
            float newHumidity = dht->readHumidity();
            float newTemperature;
            #ifdef TEMP_CELSIUS
                newTemperature = dht->readTemperature();
            #else
                newTemperature = dht->readTemperature(true);  // Fahrenheit
            #endif

            // Check if reads failed and handle errors
            if (isnan(newHumidity) || isnan(newTemperature)) {
                handleSensorReadError();
                return false;
            }

            // Update sensor values if read was successful
            SensorHumidity = newHumidity;
            SensorTemperature = newTemperature;

            // Log successful readings periodically
            logSuccessfulReading();
            return true;
        }

        void handleSensorReadError() {
            if (currentTime - lastPrintTime >= printInterval) {
                Serial.println("Error: Failed to read from DHT sensor");
                lastPrintTime = currentTime;
            }
        }

        void logSuccessfulReading() {
            if (currentTime - lastPrintTime >= printInterval) {
                Serial.printf("Temperature and Humidity read successfully: %.2f %s, %.2f%%\n",
                              SensorTemperature,
                              #ifdef TEMP_CELSIUS
                                  "°C",
                              #else
                                  "°F",
                              #endif
                              SensorHumidity);
                lastPrintTime = currentTime;
            }
        }

        void loop() {
            currentTime = millis();
            if (currentTime - lastMeasurement >= measurementInterval) {
                lastMeasurement = currentTime;  // Update time for the next reading
                
                bool success = _updateSensorData();
                // If first read failed, try conservative retries to allow sensor to stabilize
                if (!success) {
                    for (uint8_t r = 0; r < DHT_RETRIES && !success; ++r) {
                        delay(DHT_RETRY_DELAY_MS);
                        success = _updateSensorData();
                    }
                }

                if (mqtt != nullptr && mqtt->connected()) {
                    if (!mqttInitialized) {
                        _mqttInitialize();
                        mqttInitialized = true;
                    }

                    if (!isnan(SensorTemperature) && !isnan(SensorHumidity)) {
                        char tempStr[10], humidStr[10];
                        dtostrf(SensorTemperature, 1, 2, tempStr);
                        dtostrf(SensorHumidity, 1, 2, humidStr);
                        
                        mqtt->publish(mqttTemperatureTopic.c_str(), 0, true, tempStr);
                        mqtt->publish(mqttHumidityTopic.c_str(), 0, true, humidStr);
                        
                        if (currentTime - lastPrintTime >= printInterval) {
                            Serial.printf("Published to MQTT - Temperature: %s, Humidity: %s\n", tempStr, humidStr);
                            lastPrintTime = currentTime;
                        }
                    }
                } else {
                    if (currentTime - lastPrintTime >= printInterval) {
                        Serial.println("MQTT disconnected. Unable to publish DHT22 data.");
                        lastPrintTime = currentTime;
                        mqttInitialized = false;
                    }
                }
            }
        }

        void addToJsonInfo(JsonObject& root) {
            JsonObject user = root["u"];
            if (user.isNull()) user = root.createNestedObject("u");

            JsonArray temp = user.createNestedArray("Temperature");
            JsonArray humid = user.createNestedArray("Humidity");

            if (isnan(SensorTemperature) || isnan(SensorHumidity)) {
                temp.add("Sensor Error!");
                humid.add("Sensor Error!");
            } else {
                char tempStr[10], humidStr[10];
                dtostrf(SensorTemperature, 1, 2, tempStr);
                dtostrf(SensorHumidity, 1, 2, humidStr);

                temp.add(tempStr);
                #ifdef TEMP_CELSIUS
                temp.add("°C");
                #else
                temp.add("°F");
                #endif

                humid.add(humidStr);
                humid.add("%");
            }

            // Add last measurement time
            JsonArray lastMeasure = user.createNestedArray("Last Measurement");
            char timeStr[20];
            unsigned long elapsedTime = (currentTime - lastMeasurement) / 1000; // Convert to seconds
            sprintf(timeStr, "%lu seconds ago", elapsedTime);
            lastMeasure.add(timeStr);
        }

        void addToConfig(JsonObject& root) {
            JsonObject top = root.createNestedObject(FPSTR(_name));
            top[FPSTR(_enabled)] = enabled;
            // store human-friendly seconds values in config
            top[FPSTR(_measurement_interval_s)] = (measurementInterval / 1000UL);
            top[FPSTR(_retries)] = DHT_RETRIES;
            top[FPSTR(_retry_delay_s)] = (DHT_RETRY_DELAY_MS / 1000UL);
            top[FPSTR(_pin)] = dhtPin;
            top[FPSTR(_restart_note)] = F("Changing pin requires restart");
        }

        bool readFromConfig(JsonObject& root) {
            JsonObject top = root[FPSTR(_name)];
            if (top.isNull()) {
                DEBUG_PRINTLN(F("DHT22ToMQTT: No config found. Using defaults."));
                return false;
            }

            enabled = top[FPSTR(_enabled)] | enabled;
            // Read numeric values if present (fallback to existing values)
            // measurement interval: prefer seconds key, fall back to legacy ms
            if (top.containsKey(FPSTR(_measurement_interval_s))) {
                unsigned long secs = (unsigned long)top[FPSTR(_measurement_interval_s)];
                measurementInterval = secs * 1000UL;
            } else if (top.containsKey(FPSTR(_measurement_interval))) {
                measurementInterval = (unsigned long)top[FPSTR(_measurement_interval)];
            }
            // clamp measurement interval between 5s and 24h
            if (measurementInterval < 5000UL) measurementInterval = 5000UL;
            if (measurementInterval > 86400000UL) measurementInterval = 86400000UL;

            if (top.containsKey(FPSTR(_retries))) {
                uint8_t r = (uint8_t)top[FPSTR(_retries)];
                if (r > 10) r = 10;
                DHT_RETRIES = r;
            }

            // retry delay: prefer seconds key, fall back to legacy ms
            if (top.containsKey(FPSTR(_retry_delay_s))) {
                unsigned long secs = (unsigned long)top[FPSTR(_retry_delay_s)];
                DHT_RETRY_DELAY_MS = secs * 1000UL;
            } else if (top.containsKey(FPSTR(_retry_delay))) {
                DHT_RETRY_DELAY_MS = (unsigned long)top[FPSTR(_retry_delay)];
            }
            // clamp retry delay between 200ms and 10s
            if (DHT_RETRY_DELAY_MS < 200UL) DHT_RETRY_DELAY_MS = 200UL;
            if (DHT_RETRY_DELAY_MS > 10000UL) DHT_RETRY_DELAY_MS = 10000UL;

            if (top.containsKey(FPSTR(_pin))) {
                dhtPin = (uint8_t)top[FPSTR(_pin)];
            }

            return true;
        }

        uint16_t getId(){
            return USERMOD_ID_DHT22TOMQTT;
        }
};

// strings to reduce flash memory usage (used more than twice)
const char Usermod_DHT22ToMQTT::_name[]    PROGMEM = "DHT22ToMQTT";
const char Usermod_DHT22ToMQTT::_enabled[] PROGMEM = "enabled";
const char Usermod_DHT22ToMQTT::_measurement_interval[] PROGMEM = "measurementInterval";
const char Usermod_DHT22ToMQTT::_retries[] PROGMEM = "retries";
const char Usermod_DHT22ToMQTT::_retry_delay[] PROGMEM = "retryDelay";
const char Usermod_DHT22ToMQTT::_pin[] PROGMEM = "pin";
const char Usermod_DHT22ToMQTT::_measurement_interval_s[] PROGMEM = "measurementInterval_s";
const char Usermod_DHT22ToMQTT::_retry_delay_s[] PROGMEM = "retryDelay_s";
const char Usermod_DHT22ToMQTT::_restart_note[] PROGMEM = "restartNote";
