#include <Arduino.h>

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct 
{
  float pos_x;
  float pos_y;
}robotPosition;

extern void robotPositionInit(void);
extern robotPosition getRobotPosition(float prevPosX, float prevPosY, int motor1PulseCount, int motor2PulseCount, float orientationTheta);

#ifdef __cplusplus
}
#endif