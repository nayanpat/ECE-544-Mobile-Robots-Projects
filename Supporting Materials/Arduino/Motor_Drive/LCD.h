#include <Wire.h>
#include <LiquidCrystal_I2C.h>

typedef struct{

  float imuOrientiation_X;
  float robotPos_X;
  float robotPos_Y;

  float robotTargetPos_X;
  float robotTargetPos_Y;
  float imuTargetOrientiation_X;

}lcdData;

void lcdInit(void);
void printLCDScreen(lcdData data);