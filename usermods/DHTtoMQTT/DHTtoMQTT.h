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

class Usermod_DHTToMQTT : public Usermod {
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
            // Read temperature and humidity (DHT can take 250ms - 2s to read)
            float newHumidity = dht.readHumidity();
            float newTemperature;
            #ifdef TEMP_CELSIUS
                newTemperature = dht.readTemperature();
            #else
                newTemperature = dht.readTemperature(true);  // Fahrenheit
            #endif

            // Check if reads failed and handle errors
            if (isnan(newHumidity) || isnan(newTemperature)) {
                handleSensorReadError();
                return;
            }

            // Update sensor values if read was successful
            SensorHumidity = newHumidity;
            SensorTemperature = newTemperature;

            // Log successful readings periodically
            logSuccessfulReading();
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
            if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
                lastMeasurement = currentTime;  // Update time for the next reading
                
                _updateSensorData();

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

        uint16_t getId(){
            return USERMOD_ID_DHT22ToMQTT;
        }
};
