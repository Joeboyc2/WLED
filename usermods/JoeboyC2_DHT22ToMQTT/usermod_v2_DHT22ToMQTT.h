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

// Setup the Temperature Sensor
#define DHTTYPE DHT22   // DHT 22
DHT dht(DHTPIN, DHTTYPE);
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

    public:
        void setup() {
            // DHT Setup
            Serial.println("Starting DHT!");
            Serial.println("Initialising Temperature sensor.. ");
            dht.begin();
            lastMeasurement = millis();  // Initialize lastMeasurement to the boot time
        }

        void _mqttInitialize() {
            Serial.println("Setting up MQTT");
            mqttTemperatureTopic = String(mqttDeviceTopic) + "/temperature";
            mqttHumidityTopic = String(mqttDeviceTopic) + "/humidity";
            #ifdef TEMP_CELSIUS
                _createMqttSensor("temperature", mqttTemperatureTopic, "temperature", "°C");
            #else
                _createMqttSensor("temperature", mqttTemperatureTopic, "temperature", "°F");
            #endif
            _createMqttSensor("humidity", mqttHumidityTopic, "humidity", "%");
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

        void _updateSensorData() {
            // Read the temperature and humidity (DHT can take 250ms - 2s to read)
            SensorHumidity = dht.readHumidity();
            #ifdef TEMP_CELSIUS
                SensorTemperature = dht.readTemperature();
            #else
                SensorTemperature = dht.readTemperature(true);  // Fahrenheit
            #endif

            // Check if any reads failed and exit
            if (isnan(SensorHumidity) || isnan(SensorTemperature)) {
                if (currentTime - lastPrintTime >= printInterval) {
                    Serial.println("Failed to read the DHT Sensor");
                    lastPrintTime = currentTime;
                }
                return;
            }
            // Only print successful readings once per minute
            if (currentTime - lastPrintTime >= printInterval) {
                Serial.printf("Temperature and Humidity read successfully\n %f °C, %f %%\n",
                              SensorTemperature, SensorHumidity);
                lastPrintTime = currentTime;
            }
        }

        void loop() {
            currentTime = millis();
            if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
                lastMeasurement = currentTime;  // Update time for the next reading
                
                // Check if MQTT is connected
                if (mqtt != nullptr && mqtt->connected()) {
                    if (!mqttInitialized) {
                        _mqttInitialize();
                        mqttInitialized = true;
                    }

                    // Update sensor data
                    _updateSensorData();

                    // Publish temperature and humidity to MQTT
                    if (!isnan(SensorTemperature) && !isnan(SensorHumidity)) {
                        mqtt->publish(mqttTemperatureTopic.c_str(), 0, false, String(SensorTemperature).c_str());
                        mqtt->publish(mqttHumidityTopic.c_str(), 0, false, String(SensorHumidity).c_str());
                    }
                } else {
                    // Print MQTT connection status only once per minute
                    if (currentTime - lastPrintTime >= printInterval) {
                        Serial.println("Missing MQTT connection for DHT22. Not publishing data");
                        lastPrintTime = currentTime;
                    }
                    mqttInitialized = false;
                }
            }
        }

        void addToJsonInfo(JsonObject& root) {
            JsonObject user = root["u"];
            if (user.isNull()) user = root.createNestedObject("u");

            JsonArray temp = user.createNestedArray("Temperature");
            if (isnan(SensorTemperature)) {
                temp.add(" Sensor Error!");
            } else {
                temp.add(SensorTemperature);
                #ifdef TEMP_CELSIUS
                    temp.add("°C");
                #else
                    temp.add("°F");
                #endif
            }

            JsonArray humid = user.createNestedArray("Humidity");
            if (isnan(SensorHumidity)) {
                humid.add(" Sensor Error!");
            } else {
                humid.add(SensorHumidity);
                humid.add("%");
            }
        }

        uint16_t getId(){
            return USERMOD_ID_DHT22ToMQTT;
        }
};
