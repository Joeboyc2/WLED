#pragma once

#include "wled.h"
#include <Arduino.h>

// Motion settings
// Default MotionPin - can be overridden by user config
#ifndef motionInputPin
    #define motionInputPin 16
#endif

class Usermod_MotionToMQTT : public Usermod {
  private:
        bool motionDetected = LOW;
        unsigned long motionStateChange = 0;
    // Delay motion detection, this prevents LEDS's turning on after a reboot
    unsigned long motionDelay = 60000; // 1 minute delay (configurable)
        // Variable to capture the current time in the loop
        unsigned long currentTime = 0;
    bool mqttInitialized = false;
    String mqttMotionTopic = "";
        unsigned long startupTime;
        unsigned long lastPrintTime = 0;
        const unsigned long printInterval = 60000; // Print every 60 seconds
        bool enabled = true; // Enable by default
    // Configurable motion input pin
    uint8_t motionPin = motionInputPin;

        // strings to reduce flash memory usage (used more than twice)
    static const char _name[];
    static const char _enabled[];
    static const char _motion_delay[];
    static const char _pin[];
    static const char _restart_note[];
  public:
        void setup() {
            startupTime = millis();
            // Motion setup
            Serial.println("Starting Motion!");
            Serial.println("Initialising Motion sensor.. ");
            pinMode(motionPin, INPUT);
        }

        void _mqttInitialize() {
            mqttMotionTopic = String(mqttDeviceTopic) + "/motion";
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
            int currentMotionState = digitalRead(motionPin);
            
            if (currentMotionState != motionDetected) {
                motionDetected = currentMotionState;
                motionStateChange = currentTime;

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
            if (!enabled) return;

            currentTime = millis();
            
            // Check if enough time has passed since system start (motionDelay milliseconds)
            if (currentTime - startupTime >= motionDelay) {
                // Update sensor data
                _updateSensorData();  

                // Check MQTT connection and initialize if necessary
                if (mqtt != nullptr && mqtt->connected()) {
                    if (!mqttInitialized) {
                        _mqttInitialize();
                        mqttInitialized = true;
                    } 
                } else {
                    // Handle MQTT disconnection
                    if (currentTime - lastPrintTime >= printInterval) {
                        Serial.println(F("Missing MQTT connection for Motion. Not publishing data"));
                        lastPrintTime = currentTime;
                    }
                    mqttInitialized = false;
                }
            } else {
                // Print delay message only once per minute
                if (currentTime - lastPrintTime >= printInterval) {
                    Serial.println(F("Waiting for delay to pass before detecting motion..."));
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
            JsonArray motionPinArr = user.createNestedArray("Motion Sensor Pin");
            motionPinArr.add(motionPin);
        }

        void addToConfig(JsonObject& root) {
            JsonObject top = root.createNestedObject(FPSTR(_name));
            top[FPSTR(_enabled)] = enabled;
            // store seconds
            top[FPSTR(_motion_delay)] = (motionDelay / 1000UL);
            top[FPSTR(_pin)] = motionPin;
            top[FPSTR(_restart_note)] = F("Changing pin requires restart");
        }

        bool readFromConfig(JsonObject& root) {
            JsonObject top = root[FPSTR(_name)];
            if (top.isNull()) {
                DEBUG_PRINTLN(F("No config found. Creating with defaults..."));
                return false;
            }
            enabled = top[FPSTR(_enabled)] | enabled;
            // prefer seconds value
            if (top.containsKey(FPSTR(_motion_delay))) {
                unsigned long secs = (unsigned long)top[FPSTR(_motion_delay)];
                motionDelay = secs * 1000UL;
            }
            // clamp to between 1s and 1 day
            if (motionDelay < 1000UL) motionDelay = 1000UL;
            if (motionDelay > 86400000UL) motionDelay = 86400000UL;
            if (top.containsKey(FPSTR(_pin))) {
                motionPin = (uint8_t)top[FPSTR(_pin)];
            }
            return true;
        }

        uint16_t getId() {
            return USERMOD_ID_MOTIONTOMQTT;
        }
};

// strings to reduce flash memory usage (used more than twice)
const char Usermod_MotionToMQTT::_name[]    PROGMEM = "MotionToMQTT";
const char Usermod_MotionToMQTT::_enabled[] PROGMEM = "enabled";
const char Usermod_MotionToMQTT::_motion_delay[] PROGMEM = "motionDelay";
const char Usermod_MotionToMQTT::_pin[] PROGMEM = "pin";
const char Usermod_MotionToMQTT::_restart_note[] PROGMEM = "restartNote";