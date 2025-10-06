# DHT22 to MQTT Usermod

This usermod allows you to connect a DHT22 temperature and humidity sensor to your WLED device and publish its readings to MQTT for home automation integration.

## Installation

1. Copy the file `usermod_v2_DHT22ToMQTT.h` to the `wled00` directory.
2. Add the following to your `platformio_override.ini`:
```ini
build_flags = ${common.build_flags}
    -D USERMOD_DHT22TOMQTT
    -D DHTPIN=14  # Change to your DHT22 data pin
lib_deps = ${esp8266.lib_deps}
    adafruit/DHT sensor library @ ^1.4.6
    adafruit/Adafruit Unified Sensor @ ^1.1.14
```

## Configuration

### Hardware Setup
The DHT22 sensor should be connected to GPIO14 by default. You can change this by defining `DHTPIN` before including the usermod header:

```cpp
#define DHTPIN 5  // Use GPIO5 instead of default GPIO14
#include "usermod_v2_DHT22ToMQTT.h"
```

### MQTT Setup
This usermod requires MQTT to be enabled in WLED. Configure your MQTT settings in WLED's configuration panel:

1. Enable MQTT in WLED's settings
2. Configure your MQTT broker details
3. Set up a unique device name

The usermod will automatically:
- Create Home Assistant MQTT discovery topics
- Publish temperature and humidity states to:
  - `<deviceTopic>/temperature`
  - `<deviceTopic>/humidity`
- Send regular state updates every 60 seconds (configurable)

### Features
- Temperature and humidity monitoring
- Home Assistant auto-discovery support
- Configurable via WLED UI (enable/disable)
- Celsius/Fahrenheit temperature support (default: Celsius)
- Automatic MQTT reconnection handling
- Error handling for sensor read failures

### MQTT Topics
The usermod publishes to the following topics:
- Temperature topic: `<deviceTopic>/temperature` with value in °C or °F
- Humidity topic: `<deviceTopic>/humidity` with value in %
- Discovery topics:
  - `homeassistant/sensor/<deviceID>/temperature/config`
  - `homeassistant/sensor/<deviceID>/humidity/config`

## Version History

- v1.0.0 (2025-10-06)
  - Initial release
  - Basic temperature and humidity monitoring
  - MQTT integration
  - Home Assistant auto-discovery support