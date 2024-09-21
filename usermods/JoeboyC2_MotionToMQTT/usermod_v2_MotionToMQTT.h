#pragma once

#include "wled.h"
#include <Arduino.h>

// Motion settings
// Set MotionPin
#ifndef motionInputPin
  #define motionInputPin 16;
#endif

class Usermod_MotionToMQTT : public Usermod {
  private:
        bool motionDetected = false;
        bool sensorMotion = false;
        unsigned long motionStateChange = 0;
        // Delay motion detection, this prevents LEDS's turning on after a reboot
        long motionDelay = 40000;
        // Variable to capture the current time in the loop
        unsigned long currentTime = 0;
        bool mqttInitialized = false;
        String mqttMotionTopic = "";
        unsigned long nextMeasure = 0;
  public:
        void setup() {
            startupTime = millis();
            // Motion setup
            Serial.println("Starting Motion!");
            Serial.println("Initialising Motion sensor.. ");
            pinMode(motionInputPin, INPUT);
        }

        void _mqttInitialize() {
            mqttMotionTopic = String(mqttDeviceTopic) + "/motion";

            String m = String("homeassistant/binary_sensor/") + mqttClientID + "/motion/config";

            _createMqttSensor("motion", mqttMotionTopic, "motion");
        }

        void _createMqttSensor(const String &name, const String &topic, const String &deviceClass) {
            String m = String("homeassistant/binary_sensor/") + mqttClientID + "/" + name + "/config";

            StaticJsonDocument<300> doc;

            doc["name"] = name;
            doc["state_topic"] = topic;
            doc["unique_id"] = String(mqttClientID) + name;
            doc["device_class"] = deviceClass;
            doc["expire_after"] = 1800;

            JsonObject device = doc.createNestedObject("device"); // attach the sensor to the same device
            device["identifiers"] = String("wled-sensor-") + mqttClientID;
            device["manufacturer"] = "Aircoookie";
            device["model"] = "WLED";
            device["sw_version"] = VERSION;
            device["name"] = mqttClientID;

            String motion;
            serializeJson(doc, motion);
            Serial.println(m);
            Serial.println(motion);

            mqtt->publish(m.c_str(), 0, false, motion.c_str());
        }

        void _updateSensorData() {
        // Detect motion and publish message to MQTT
            motionDetected = digitalRead(motionInputPin);
        // If motion is detected, publish message
            if(motionDetected == HIGH) {
            // Has motion already been triggered
                if (!sensorMotion) {
                    Serial.println("Motion detected!");
                    sensorMotion = true;
                    mqtt->publish(mqttMotionTopic.c_str(), 0, false, "ON");
                } else if (sensorMotion) {
                    Serial.println("Motion Ended!");
                    sensorMotion = false;
                    mqtt->publish(mqttMotionTopic.c_str(), 0, false, "OFF");
                }
            }
        }        

        // gets called every time WiFi is (re-)connected.
        void connected() {
            nextMeasure = millis() + 5000; // Schedule next measure in 5 seconds
        }
    
        void loop() {
            currentTime = millis();
            // Check if enough time has passed since system start (motionDelay milliseconds)
            if (currentTime - startupTime >= motionDelay) {
            // Publish reading to JSON & HomeAssistant API 
                if (mqtt != nullptr && mqtt->connected()) {
                    if (!mqttInitialized) {
                        _mqttInitialize();
                        mqttInitialized = true;
                    } 
                // Update sensor data
                    _updateSensorData();  
                } else {
                    Serial.println("Missing MQTT connection. Not publishing data");
                    mqttInitialized = false;
                }
            } else {
                // Print this if delay has not passed, useful for debugging
                Serial.println("Waiting for delay to pass before detecting motion...");
            }
            }
        }

        void addToJsonInfo(JsonObject& root) {
            JsonObject user = root["u"];
            if (user.isNull()) user = root.createNestedObject("u");

            JsonArray mot = user.createNestedArray("Motion");
            // Add Motion state to Json API
            if (isnan(motionDetected)) {
                mot.add(" Sensor Error!");
                return;
            }

            if (motionDetected == HIGH){
                mot.add(" Motion Detected!");
            }else{
                mot.add(" No Motion Detected");
            }

            JsonArray lastmot = user.createNestedArray("Last Motion Event");

            if (motionDetected == 0) {
                lastmot.add(" No Motion Yet");
                return;
            }else{
                lastmot.add(motionStateChange);
            }
        }

        uint16_t getId(){
            return USERMOD_ID_MotionToMQTT;
        }
};