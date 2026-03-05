#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

void imuInit(void);
sensors_event_t imuOrientationData(void);
sensors_event_t imuAngVelocityData(void);
