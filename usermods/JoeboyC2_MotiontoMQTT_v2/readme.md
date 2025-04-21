# MotiontoMQTT usermod

Reads an attached PIR sensor and displays the status of it in Info section of the web UI, updates will also be published to the `/motion` MQTT topic, when MQTT is enabled.

Maintained by @joeboyc2

## Installation

Add `JoeboyC2_DHTtoMQTT_v2` to `custom_usermods` in your platformio_override.ini

Example **platformio_override.ini**:

```ini
[env:nodemcuv2_5v_Controller_DHT22]
board = nodemcuv2
custom_usermods = JoeboyC2_DHTtoMQTT_v2
```

### Define Your Options

* `MOTION_PIN` - specify the pin number to use, defaults to pin 16 on esp8266 based boards

N.B. The motion pin number can be changed via the Usermods settings page along with an option ti disable the user mod if desired.

## Change Log

2025-04
* Initial release of v2 usermod