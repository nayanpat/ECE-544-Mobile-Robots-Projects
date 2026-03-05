#include "LCD.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);

void lcdInit(void){

  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

  // lcd.begin();
  lcd.init(); 
  lcd.backlight();
  // lcd.clear();  
  lcd.setCursor (0,0); //
  lcd.print("Robojax LCD2004 Test"); 
  
  delay(10);
}

void printLCDScreen(lcdData data){

  char X[5], Y[5], Theta1[5];  

  lcd.clear();  

  lcd.setCursor (0, 0); //character zero, line 0
  lcd.print("Set Val"); // print text  
  lcd.setCursor (8, 0); //character 4, line 0
  lcd.print("|Actual Val"); // print text 
  delay(0.01);  

  lcd.setCursor (0, 1); //character zero, line 0
  lcd.print("Xs:"); // print text  
  lcd.setCursor (4, 1); //character 4, line 0
  dtostrf(data.robotTargetPos_X, 3, 2, X);
  lcd.print(X); // print text 
  lcd.setCursor (10, 1); //character zero, line 0
  lcd.print("Xa:"); // print text  
  lcd.setCursor (13, 1); //character 14, line 0
  dtostrf(data.robotPos_X, 3, 2, X);
  lcd.print(X); // print text 
  delay(0.01);  

  lcd.setCursor (0, 2); //character zero, line 1  
  lcd.print("Ys:"); // print text  
  lcd.setCursor (4, 2); //character 4, line 1
  dtostrf(data.robotTargetPos_Y, 3, 2, Y);
  lcd.print(Y); // print text
  lcd.setCursor (10, 2); //character zero, line 1  
  lcd.print("Ya:"); // print text  
  lcd.setCursor (13, 2); //character 14, line 1
  dtostrf(data.robotPos_Y, 3, 2, Y);
  lcd.print(Y); // print text
  delay(0.01);     

  lcd.setCursor (0, 3); //character zero, line 2  
  lcd.print("Ts:"); // print text  
  lcd.setCursor (4, 3); //character 4, line 0
  dtostrf(data.imuTargetOrientiation_X, 4, 2, Theta1);
  lcd.print(Theta1); // print text
  lcd.setCursor (10, 3); //character zero, line 2  
  lcd.print("Ta:"); // print text  
  lcd.setCursor (14, 3); //character 4, line 0
  dtostrf(data.imuOrientiation_X, 4, 2, Theta1);
  lcd.print(Theta1); // print text
  delay(0.01); 
}