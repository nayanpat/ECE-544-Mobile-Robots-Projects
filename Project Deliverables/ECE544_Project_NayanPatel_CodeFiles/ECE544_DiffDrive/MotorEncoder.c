#include "MotorEncoder.h"

static int posCountEn1 = 0;
static int posCountEn2 = 0;

extern void motorEncoderInit(void){
  
  pinMode(ENCA_M1, INPUT);  // sets the Encoder_output_A pin as the input
  pinMode(ENCB_M1, INPUT);  // sets the Encoder_output_B pin as the input
  pinMode(OUT1_M1, OUTPUT);
  pinMode(OUT2_M1, OUTPUT);
  pinMode(PWM_OUT_M1, OUTPUT);

  pinMode(ENCA_M2, INPUT);  // sets the Encoder_output_A pin as the input
  pinMode(ENCB_M2, INPUT);  // sets the Encoder_output_B pin as the input
  pinMode(OUT1_M2, OUTPUT);
  pinMode(OUT2_M2, OUTPUT);
  pinMode(PWM_OUT_M2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_M1), readEncoderEn1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2), readEncoderEn2, RISING);  
}

extern void setMotor(int dir, int pwmVal, int pwm, int out1, int out2) {
  
  analogWrite(pwm, pwmVal);

  if (dir == 1) {
    digitalWrite(out1, HIGH);
    digitalWrite(out2, LOW);
  } else if (dir == -1) {
    digitalWrite(out1, LOW);
    digitalWrite(out2, HIGH);
  } 
  else {
    digitalWrite(out1, LOW);
    digitalWrite(out2, LOW);
  }
}

extern void readEncoderEn1(void) {

  int b = digitalRead(ENCB_M1);

  if (b > 0) {
    posCountEn1++;
  } 
  else {
    posCountEn1--;
  }
}
extern void readEncoderEn2(void) {

  int b = digitalRead(ENCB_M2);

  if (b > 0) {
    posCountEn2++;
  } 
  else {
    posCountEn2--;
  }
}

extern int getPosCountEn1(void) {
  
  return -posCountEn1;
}

extern int getPosCountEn2(void) {
  
  return posCountEn2;
}