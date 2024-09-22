#pragma once

#include "wled.h"
#include <Arduino.h>

// Motion settings
// Set MotionPin
#ifndef motionInputPin
  #define motionInputPin 16
#endif

class Usermod_MotionToMQTT : public Usermod {
  private:
        bool motionDetected = LOW;
        unsigned long motionStateChange = 0;
        // Delay motion detection, this prevents LEDS's turning on after a reboot
        long motionDelay = 60000;
        // Variable to capture the current time in the loop
        unsigned long currentTime = 0;
        bool mqttInitialized = false;
        String mqttMotionTopic = "";
        unsigned long startupTime;
        unsigned long lastPrintTime = 0;
        const unsigned long printInterval = 60000; // Print every 60 seconds
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
            int currentMotionState = digitalRead(motionInputPin);
            
            if (currentMotionState != motionDetected) {
                motionDetected = currentMotionState;
                motionStateChange = millis();

                const char* motionStatus = (motionDetected == HIGH) ? "ON" : "OFF";
                Serial.printf("Motion %s!\n", motionStatus);

                if (mqtt != nullptr && mqtt->connected()) {
                    mqtt->publish(mqttMotionTopic.c_str(), 0, true, motionStatus);
                } else {
                    Serial.println("MQTT not connected. Unable to publish motion status.");
                }
            }
        }
            
        void loop() {
            currentTime = millis();
            // Check if enough time has passed since system start (motionDelay milliseconds)
            if (currentTime - startupTime >= motionDelay) {
                // Update sensor data
                _updateSensorData();  

                // Publish reading to JSON & HomeAssistant API 
                if (mqtt != nullptr && mqtt->connected()) {
                    if (!mqttInitialized) {
                        _mqttInitialize();
                        mqttInitialized = true;
                    } 
                } else {
                    // Print MQTT connection status only once per minute
                    if (currentTime - lastPrintTime >= printInterval) {
                        Serial.println("Missing MQTT connection for Motion. Not publishing data");
                        lastPrintTime = currentTime;
                    }
                    mqttInitialized = false;
                }
            } else {
                // Print delay message only once per minute
                if (currentTime - lastPrintTime >= printInterval) {
                    Serial.println("Waiting for delay to pass before detecting motion...");
                    lastPrintTime = currentTime;
                }
            }
        }

        void addToJsonInfo(JsonObject& root) {
            JsonObject user = root["u"];
            if (user.isNull()) user = root.createNestedObject("u");

            // Add Motion state to Json API
            JsonArray motionState = user.createNestedArray("Motion State");
            motionState.add(motionDetected ? "Detected" : "Not Detected");

            // Add Last Motion Event to Json API
            JsonArray lastMotionEvent = user.createNestedArray("Last Motion Event");
            if (motionStateChange == 0) {
                lastMotionEvent.add("No motion detected yet");
            } else {
                char timeStr[20];
                unsigned long elapsedTime = (currentTime - motionStateChange) / 1000; // Convert to seconds
                sprintf(timeStr, "%lu seconds ago", elapsedTime);
                lastMotionEvent.add(timeStr);
            }

            // Add Motion Input Pin to Json API
            JsonArray motionPin = user.createNestedArray("Motion Sensor Pin");
            motionPin.add(motionInputPin);
        }

        uint16_t getId(){
            return USERMOD_ID_MotionToMQTT;
        }
};