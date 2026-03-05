#include <Arduino.h>

#ifdef __cplusplus
 extern "C" {
#endif

/* Motor/Encoder 1 pin definition */
#define ENCA_M1 2         // Encoder output A connected to pin no. 2
#define ENCB_M1 3         // Encoder output B connected to pin no. 3
#define PWM_OUT_M1 5      // PWM output pin
#define OUT1_M1 7         // Digital output pin number for direction change
#define OUT2_M1 6         // Digital output pin number for direction change
/* Motor/Encoder 2 pin definition */
#define ENCA_M2 12         // Encoder output A connected to pin no. 12
#define ENCB_M2 13         // Encoder output B connected to pin no. 13
#define PWM_OUT_M2 9        // PWM output pin
#define OUT1_M2 11         // Digital output pin number for direction change
#define OUT2_M2 10         // Digital output pin number for direction change

#define HIGH 1      // High state on DO pin
#define LOW 0       // Low state on DO pin

extern void motorEncoderInit(void);
extern void setMotor(int dir, int pwmVal, int pwm, int out1, int out2);
extern void readEncoderEn1(void);
extern void readEncoderEn2(void);
extern int getPosCountEn1(void);
extern int getPosCountEn2(void);

#ifdef __cplusplus
}
#endif