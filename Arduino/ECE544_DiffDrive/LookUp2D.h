#include <Arduino.h>

#ifdef __cplusplus
 extern "C" {
#endif

extern float interpolation2D(float **radPerSecToPWMTable, int row, int col, float RPSInputVal);

#ifdef __cplusplus
}
#endif