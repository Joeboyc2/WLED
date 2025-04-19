# Temperature usermod

Reads an attached DHT11,21,22, the temperature and humidity valueswill be  displayed in both the Info section of the web UI as well as published to the `/temperature` & `/humidity` MQTT topics, if enabled.  

If temperature sensor is not detected during boot, this usermod will be disabled.

Maintained by @joeboyc2

## Installation

Add `DHTtoMQTT` to `custom_usermods` in your platformio_override.ini.

Example **platformio_override.ini**:

```ini
[env:nodemcuv2_5v_Controller_DHT22]
board = nodemcuv2
custom_usermods = DHTtoMQTT
```

### Define Your Options

* `DHTPIN` - specify the pin number to use, defaults to pin 14 on esp8266 based boards
* `DHTTYPE` - specify the type of DHT you are using, values can be 11, 21 or 22, defaults to DHT22

All parameters can be configured at runtime via the Usermods settings page, including pin, temperature in degrees Celsius or Fahrenheit and measurement interval.

## Change Log

2025-04
* Initial release of v2 usermod