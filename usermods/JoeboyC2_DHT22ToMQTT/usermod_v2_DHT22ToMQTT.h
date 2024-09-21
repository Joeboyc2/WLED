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
        unsigned long lastMeasurement = 40000;
        bool mqttInitialized = false;
        float SensorTemperature = 1;
        float SensorHumidity = 1;
        String mqttTemperatureTopic = "";
        String mqttHumidityTopic = "";
        unsigned long nextMeasure = 0;
        // Variable to capture the current time in the loop
        unsigned long currentTime = 0;

    public:
        void setup() {
            // DHT Setup
            Serial.println("Starting DHT!");
            Serial.println("Initialising Temperature sensor.. ");
            dht.begin();
        }

        void _mqttInitialize() {
            Serial.println("Setting up MQTT");
            mqttTemperatureTopic = String(mqttDeviceTopic) + "/temperature";
            mqttHumidityTopic = String(mqttDeviceTopic) + "/humidity";

            String t = String("homeassistant/sensor/") + mqttClientID + "/temperature/config";

            _createMqttSensor("temperature", mqttTemperatureTopic, "temperature", "°C");
            _createMqttSensor("humidity", mqttHumidityTopic, "humidity", "%");
        }

        void _createMqttSensor(const String &name, const String &topic, const String &deviceClass, const String &unitOfMeasurement){
            String t = String("homeassistant/sensor/") + mqttClientID + "/" + name + "/config";

            StaticJsonDocument<300> doc;

            doc["name"] = name;
            doc["state_topic"] = topic;
            doc["unique_id"] = String(mqttClientID) + name;
            if (unitOfMeasurement != "")
                doc["unit_of_measurement"] = unitOfMeasurement;
            if (deviceClass != "")
                doc["device_class"] = deviceClass;
            doc["expire_after"] = 1800;

            JsonObject device = doc.createNestedObject("device"); // attach the sensor to the same device
            device["identifiers"] = String("wled-sensor-") + mqttClientID;
            device["manufacturer"] = "Aircoookie";
            device["model"] = "WLED";
            device["sw_version"] = VERSION;
            device["name"] = mqttClientID;

            String temp;
            serializeJson(doc, temp);
            Serial.println(t);
            Serial.println(temp);

            mqtt->publish(t.c_str(), 0, false, temp.c_str());
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
            if (currentTime - lastMeasurement > MEASUREMENT_INTERVAL) {
                if (SensorTemperature || SensorHumidity) {
                // Publish reading to JSON & HomeAssistant API
                    lastMeasurement = millis();   
                        if (mqtt != nullptr && mqtt->connected()) {
                            if (!mqttInitialized) {
                                _mqttInitialize();
                                mqttInitialized = true;
                            } 
                            // Update sensor data
                               Serial.println("Updating Sensors");
                                _updateSensorData();  
                            // Create string populated with user defined device topic from the UI and then read temperature and humidity.
                            // Then publish to MQTT server.
                                mqtt->publish(mqttTemperatureTopic.c_str(), 0, false, String(SensorTemperature).c_str());
                                mqtt->publish(mqttHumidityTopic.c_str(), 0, false, String(SensorHumidity).c_str());
                        } else {
                            Serial.println("Missing MQTT connection. Not publishing data");
                            mqttInitialized = false;
                        }
                }
            }
        }
 
    void addToJsonInfo(JsonObject& root) {
        JsonObject user = root["u"];
        if (user.isNull()) user = root.createNestedObject("u");
            JsonArray temp = user.createNestedArray("Temperature");
        if (isnan(SensorTemperature)) {
          // temp.add(0);
            temp.add(" Sensor Error!");
            return;
        }

        temp.add(SensorTemperature);
        #ifdef TEMP_CELSIUS
            temp.add("°C");
        #else
            temp.add("°F");
        #endif

        JsonArray humid = user.createNestedArray("Humidity");
        if (isnan(SensorHumidity)) {
          // humid.add(0);
            humid.add(" Sensor Error!");
            return;
        }
        humid.add(SensorHumidity);
    }

    uint16_t getId(){
      return USERMOD_ID_DHT22ToMQTT;
    }
};