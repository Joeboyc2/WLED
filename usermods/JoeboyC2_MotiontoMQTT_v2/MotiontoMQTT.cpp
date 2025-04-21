#include "wled.h"

#ifdef WLED_DISABLE_MQTT
#error "This user mod requires MQTT to be enabled."
#endif

// Motion sensor configuration
#ifndef MOTION_PIN
#define MOTION_PIN 16
#endif

// The frequency to check sensor
#ifndef MOTION_MEASUREMENT_INTERVAL
#define MOTION_MEASUREMENT_INTERVAL 1000  // Check every second
#endif

// How many seconds after boot to take first measurement
#ifndef MOTION_FIRST_MEASUREMENT_AT
#define MOTION_FIRST_MEASUREMENT_AT 60000  // 1 minute delay
#endif

class UsermodMotionToMQTT : public Usermod {
  private:
    unsigned long nextReadTime = 0;
    unsigned long lastReadTime = 0;
    unsigned long lastStateChange = 0;
    bool motionState = false;
    bool initializing = true;
    bool disabled = false;
    char mqttMotionTopic[64];
    bool mqttInitialized = false;
    bool sensorError = false;
    unsigned long lastSuccessfulRead = 0;
    const unsigned long ERROR_TIMEOUT = 10000; // 10 seconds timeout for readings

    // Config variables
    uint8_t motionPin = MOTION_PIN; // default from build flag
    bool enabled = true;

    // For configuration
    bool initDone = false;

  public:
    void setup() {
      pinMode(motionPin, INPUT);
      nextReadTime = millis() + MOTION_FIRST_MEASUREMENT_AT;
      lastReadTime = millis();

      // Initialize MQTT topic
      snprintf(mqttMotionTopic, sizeof(mqttMotionTopic), "%s/motion", mqttDeviceTopic);
    }

    void loop() {
      if (!enabled || disabled) return;
      if (millis() < nextReadTime) return;

      bool currentState = digitalRead(motionPin);
      lastSuccessfulRead = millis();
      sensorError = false;

      // Debug output for motion state
      if (currentState != motionState || (millis() - lastReadTime) >= 5000) {  // Print every 5 seconds or on change
        DEBUG_PRINTF("Motion Pin %d State: %s (Previous: %s)\n", 
          motionPin, 
          currentState ? "HIGH" : "LOW",
          motionState ? "HIGH" : "LOW"
        );
        lastReadTime = millis();
      }

      if (currentState != motionState) {
        motionState = currentState;
        lastStateChange = millis();

        // Handle MQTT if connected
        if (WLED_MQTT_CONNECTED) {
          if (!mqttInitialized) {
            publishMqttDeviceConfig();
            mqttInitialized = true;
          }
          publishSensorData();
        }
      }

      // Check for sensor errors
      if ((millis() - lastSuccessfulRead) > ERROR_TIMEOUT) {
        sensorError = true;
        DEBUG_PRINTLN(F("Motion sensor not responding!"));
        if (WLED_MQTT_CONNECTED) {
          mqtt->publish(mqttMotionTopic, 0, true, "error");
        }
      }

      nextReadTime = millis() + MOTION_MEASUREMENT_INTERVAL;
      initializing = false;
    }

    void publishMqttDeviceConfig() {
      char configTopic[128];
      snprintf(configTopic, sizeof(configTopic), 
               "homeassistant/binary_sensor/%s/motion/config", mqttClientID);

      StaticJsonDocument<300> doc;
      doc["name"] = "motion";
      doc["state_topic"] = mqttMotionTopic;
      doc["unique_id"] = String(mqttClientID) + "motion";
      doc["device_class"] = "motion";
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
      const char* state = motionState ? "ON" : "OFF";
      mqtt->publish(mqttMotionTopic, 0, true, state);
    }

    void addToJsonInfo(JsonObject& root) {
      if (disabled || !enabled) return;
      
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray motion = user.createNestedArray("Motion");
      JsonArray lastEvent = user.createNestedArray("LastMotion");

      if (initializing) {
        motion.add("Initializing ");
        motion.add((nextReadTime - millis()) / 1000);
        return;
      }

      if (sensorError) {
        motion.add("Error");
        motion.add("Sensor not responding");
        return;
      }

      motion.add(motionState ? "Detected" : "Clear");
      
      if (lastStateChange > 0) {
        char timeStr[20];
        unsigned long elapsedTime = (millis() - lastStateChange) / 1000;
        sprintf(timeStr, "%lu sec ago", elapsedTime);
        lastEvent.add(timeStr);
      }
    }

    void appendConfigData() {
      oappend(SET_F("addInfo('Motion-en',{type:'bool',caption:'Enabled?',val:"));
      oappend(enabled ? "true" : "false");
      oappend(SET_F("});"));

      oappend(SET_F("addInfo('Motion-cfg',{type:'number',caption:'Motion_Pin',min:0,max:40,val:"));
      oappend(String(motionPin).c_str());
      oappend(SET_F("});"));
    }

    bool readFromConfig(JsonObject& root) {
      JsonObject top = root[FPSTR("Motion")];
      
      if (top.isNull()) {
        DEBUG_PRINTLN(F("No config found. Creating new."));
        return false;
      }

      bool configComplete = !top[FPSTR("pin")].isNull();

      if (!initDone) {
        // first time init
        uint8_t newMotionPin = motionPin;
        newMotionPin = top[FPSTR("pin")] | motionPin;

        if (newMotionPin != motionPin) {
          motionPin = newMotionPin;
          pinMode(motionPin, INPUT);
        }

        enabled = top[FPSTR("enabled")] | enabled;

        initDone = true;
      } else {
        // changing config
        if (!top[FPSTR("pin")].isNull()) {
          motionPin = top[FPSTR("pin")] | motionPin;
          pinMode(motionPin, INPUT);
        }

        if (!top[FPSTR("enabled")].isNull()) {
          enabled = top[FPSTR("enabled")] | enabled;
        }
      }
      
      return configComplete;
    }

    void addToConfig(JsonObject& root) {
      JsonObject top = root.createNestedObject(F("Motion"));
      top[FPSTR("pin")] = motionPin;
      top[FPSTR("enabled")] = enabled;
    }

    uint16_t getId() {
      return USERMOD_ID_MOTION;
    }
};

static UsermodMotionToMQTT motionMqtt;
REGISTER_USERMOD(motionMqtt);