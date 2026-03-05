#include <Arduino.h>

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct 
{
  float LinVel;
  float AngVel;
}refVelocity;

typedef struct 
{
  float currPos_x;
  float currPos_y;
  float currOri_theta;  
}currentRobotPosition;

typedef struct 
{
  float goalPos_x;
  float goalPos_y;
  float goalOri_theta;  
}targetRobotPosition;

typedef struct
{
  float k_rho;
  float k_alpha;
  float k_beta;
}feedbackCntrlrTuneUpVals;

extern refVelocity calcRefVelocity(currentRobotPosition currentVals, targetRobotPosition desiredVals, feedbackCntrlrTuneUpVals tuneUpVals);

#ifdef __cplusplus
}
#endif
