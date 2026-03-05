#include <Arduino.h>
#include "Feedback_Controller.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct 
{
  float wheelSpeedLeft;
  float wheelSpeedRight;
}wheelSpeedCommand;

extern wheelSpeedCommand calWheelSpeed(refVelocity refVelVals);

#ifdef __cplusplus
}
#endif