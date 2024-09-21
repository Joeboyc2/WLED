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
        //set last reading as "40 sec before boot", so first reading is taken after 20 sec
        unsigned long lastMeasurement = 0; // Initialize to 0, will be set in setup()
        bool mqttInitialized = false;
        float SensorTemperature = NAN;
        float SensorHumidity = NAN;
        String mqttTemperatureTopic = "";
        String mqttHumidityTopic = "";
        unsigned long currentTime = 0;

    public:
        void setup() {
            lastMeasurement = millis();  // Initialize lastMeasurement to the boot time
            // DHT Setup
            Serial.println("Starting DHT!");
            Serial.println("Initialising Temperature sensor.. ");
            dht.begin();
        }

        void _mqttInitialize() {
            Serial.println("Setting up MQTT");
            mqttTemperatureTopic = String(mqttDeviceTopic) + "/temperature";
            mqttHumidityTopic = String(mqttDeviceTopic) + "/humidity";


            _createMqttSensor("temperature", mqttTemperatureTopic, "temperature", "°C");
            _createMqttSensor("humidity", mqttHumidityTopic, "humidity", "%");
        }

        void _createMqttSensor(const String &name, const String &topic, const String &deviceClass, const String &unitOfMeasurement){
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
            // Read the temperature and humidity, this can take between 250 milli to 2 seconds
            SensorHumidity = dht.readHumidity();
            #ifdef TEMP_CELSIUS
                SensorTemperature = dht.readTemperature();
            #else
                SensorTemperature = dht.readTemperature(true);
            #endif
            // Check if any reads failed and exit
            if (isnan(SensorHumidity) || isnan(SensorTemperature)) {
        // Log Error to console
                Serial.println("Failed to read the DHT Sensor");
                return;
            }
            Serial.printf("Temperature and Humidity read successful\n %f °C, %f ",
                        SensorTemperature, SensorHumidity);
        }
        // gets called every time WiFi is (re-)connected.
        void connected() {
            nextMeasure = millis() + 5000; // Schedule next measure in 5 seconds
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
                    Serial.println("Missing MQTT connection. Not publishing data");
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