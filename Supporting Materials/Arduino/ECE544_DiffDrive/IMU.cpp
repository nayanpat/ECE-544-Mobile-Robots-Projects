#include "IMU.h"

#define BNO055_I2C_ID             (int8_t)55
#define BNO055_I2C_ADDRESS        (uint8_t)0x28

Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_I2C_ID, BNO055_I2C_ADDRESS, &Wire);

void imuInit(void){
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(10);

  bno.setExtCrystalUse(true);
}

sensors_event_t imuOrientationData(void){
  
  /* Get a new sensor event */
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
 
  return orientationData;  
}

sensors_event_t imuAngVelocityData(void){
  
  /* Get a new sensor event */
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
 
  return angVelocityData;  
}