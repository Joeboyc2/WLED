#include "wled.h"
#include "usermod_v2_DHT22ToMQTT.h"

static Usermod_DHT22ToMQTT dht22_to_mqtt;
REGISTER_USERMOD(dht22_to_mqtt);
