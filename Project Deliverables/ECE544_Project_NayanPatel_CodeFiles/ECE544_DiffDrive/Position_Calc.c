#include "Position_Calc.h"

#define PI (float)(22.0 / 7.0)
#define WHEEL_DIAMETER (float)65.0  // in mm
#define WHEEL_BASE (float)185.0     // in mm
#define NO_OF_PULSES_PER_REV (float)11.0
#define MOTOR_GEAR_RATIO (float)47.5

static float odometryConversionFactor = 0.0;  // Conversion factor that translates encoder pulses into linear wheel displacement
static float degToRadConversionFactor = 0.0;

extern void robotPositionInit(void) {
  odometryConversionFactor = (PI * WHEEL_DIAMETER) / (MOTOR_GEAR_RATIO * NO_OF_PULSES_PER_REV);
  degToRadConversionFactor = PI / 180.0;
}

extern robotPosition getRobotPosition(float prevPosX, float prevPosY, int motor1PulseCount, int motor2PulseCount, float orientationTheta) {

  robotPosition currentPos;
  float incTravelDistanceWheelLeft = 0.0;   // Incremental distance traveled by each wheel, left
  float incTravelDistanceWheelRight = 0.0;  // Incremental distance traveled by each wheel, right
  float incLinearDisplacement = 0.0;        // Incremental Linear Displacement of the robot's center point C
  float currentOrientationInRad = 0.0;
  float incOrientation = 0.0;
  float relativeOrientation = 0.0;
  static float prevRelativeOrientation;

  incTravelDistanceWheelLeft = odometryConversionFactor * (float)motor1PulseCount;
  incTravelDistanceWheelRight = odometryConversionFactor * (float)motor2PulseCount;

  incLinearDisplacement = (incTravelDistanceWheelLeft + incTravelDistanceWheelRight) * 0.5;

  incLinearDisplacement = incLinearDisplacement / 10;

  // incOrientation = (incTravelDistanceWheelRight - incTravelDistanceWheelLeft) / WHEEL_BASE;

  // relativeOrientation = prevRelativeOrientation + incOrientation;

  relativeOrientation = (360 - orientationTheta) * degToRadConversionFactor;

  currentPos.pos_x = (prevPosX + incLinearDisplacement * cos(relativeOrientation));
  currentPos.pos_y = (prevPosY + incLinearDisplacement * sin(relativeOrientation));

  prevRelativeOrientation = orientationTheta;

  return currentPos;
}