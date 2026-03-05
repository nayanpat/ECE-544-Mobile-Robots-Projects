#include "Inverse_Kinematics.h"

#define WHEEL_BASE            (float)185.0 // in mm
#define WHEEL_RADIUS          (float)(65.0/2.0) // in mm

extern wheelSpeedCommand calWheelSpeed(refVelocity refVelVals){
  wheelSpeedCommand whlSpd = {0};

  whlSpd.wheelSpeedLeft = (1/WHEEL_RADIUS) * (refVelVals.LinVel - ((refVelVals.AngVel * WHEEL_BASE) * 0.5)); 
  whlSpd.wheelSpeedRight = (1/WHEEL_RADIUS) * (refVelVals.LinVel + ((refVelVals.AngVel * WHEEL_BASE) * 0.5)); 

  return whlSpd;
}
