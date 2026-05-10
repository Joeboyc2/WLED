#include "wled.h"
#include "usermod_v2_MotionToMQTT.h"

static Usermod_MotionToMQTT motion_to_mqtt;
REGISTER_USERMOD(motion_to_mqtt);
