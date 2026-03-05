#include "LookUp2D.h"

extern float interpolation2D(float **radPerSecToPWMTable, int row, int col, float RPSInputVal) {

  float retVal = 0.0;
  float distance = 0.0;

  /* If the input is lower than first value or higher than last value, extropolate */
  if (RPSInputVal <= radPerSecToPWMTable[0][0]) {
    retVal = radPerSecToPWMTable[0][1];
  } else if (RPSInputVal >= radPerSecToPWMTable[row - 1][0]) {
    retVal = radPerSecToPWMTable[row - 1][1];
  } else {
    /* Search from first element to second last element */
    for (int i = 1; i < row - 1; i++) {

      if (RPSInputVal < radPerSecToPWMTable[i][0] && RPSInputVal >= radPerSecToPWMTable[i - 1][0]) {
        if (RPSInputVal == radPerSecToPWMTable[i - 1][0]) {
          retVal = radPerSecToPWMTable[i - 1][1];
        } else {
          distance = (RPSInputVal - radPerSecToPWMTable[i - 1][0]) / (radPerSecToPWMTable[i][0] - radPerSecToPWMTable[i - 1][0]);
          retVal = distance * (radPerSecToPWMTable[i][1] - radPerSecToPWMTable[i - 1][1]) + radPerSecToPWMTable[i - 1][1];
        }
      }
      else{
        // do nothing
      }
    }
  }
  return retVal;
}