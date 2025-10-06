# Motion Sensor to MQTT Usermod

This usermod allows you to connect a motion sensor to your WLED device and publish its state to MQTT for home automation integration.

## Installation

1. Copy the file `usermod_v2_MotionToMQTT.h` to the `wled00` directory.
2. Register the usermod by adding `#include "usermod_v2_MotionToMQTT.h"` in the top and `registerUsermod(new Usermod_MotionToMQTT());` in the bottom of `usermods_list.cpp`.

## Configuration

### Hardware Setup
The motion sensor should be connected to GPIO16 by default. You can change this by defining `motionInputPin` before including the usermod header:

```cpp
#define motionInputPin 5  // Use GPIO5 instead of default GPIO16
#include "usermod_v2_MotionToMQTT.h"
```

### MQTT Setup
This usermod requires MQTT to be enabled in WLED. Configure your MQTT settings in WLED's configuration panel:

1. Enable MQTT in WLED's settings
2. Configure your MQTT broker details
3. Set up a unique device name

The usermod will automatically:
- Create a Home Assistant MQTT discovery topic
- Publish motion states to `<deviceTopic>/motion`
- Send regular state updates

### Features
- Motion detection and MQTT state publishing
- Home Assistant auto-discovery support
- Configurable via WLED UI (enable/disable)
- 1-minute startup delay to prevent false triggers on boot
- Automatic MQTT reconnection handling

### MQTT Topics
The usermod publishes to the following topics:
- State topic: `<deviceTopic>/motion` with payload `ON` or `OFF`
- Discovery topic: `homeassistant/binary_sensor/<deviceID>/motion/config`

## Version History

- v1.0.0 (2025-10-05)
  - Initial release
  - Basic motion detection and MQTT publishing
  - Home Assistant integration