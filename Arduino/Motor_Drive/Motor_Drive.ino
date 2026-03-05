#include "LCD.h"
#include "MotorEncoder.h"
#include "IMU.h"
#include "Position_Calc.h"
#include "Inverse_Kinematics.h"
#include "LookUp2D.h"

#define NO_OF_PULSES_PER_REV      (float)11.0
#define MOTOR_GEAR_RATIO          (float)47.5
#define FORWARD_DIRECTION         (int)1
#define REVERSE_DIRECTION         (int)-1
#define PI                        (float)(22.0/7.0)
#define MIN_SPEED                 (int)120
#define SATURATE(x, MAX)          (int)(x > MAX ? (x = MAX):x)
#define ROW                       (int)52;
#define COLUMN                    (int)2;

sensors_event_t orientationData;
sensors_event_t angVelocityData;
float initPoseAngle = 45.0;

float radPerSecToPWMTable[ROW][COLUMN] =  { {0, 0},
                                            {0, 5},
                                            {0, 10},
                                            {0, 15},
                                            {0, 20},
                                            {0, 25},
                                            {0, 30},
                                            {0, 35},
                                            {0, 40},
                                            {0, 45},
                                            {0, 50},
                                            {0, 55},
                                            {0, 60},
                                            {0, 65},
                                            {0.001047198, 70},
                                            {0.001047198, 75},
                                            {0.001047198, 80},
                                            {0.001047198, 85},
                                            {0.001047198, 90},
                                            {0.00523599, 95},
                                            {0.01570797, 100},
                                            {0.161268492, 105},
                                            {0.082728642, 110},
                                            {0.122522166, 115},
                                            {0.165457284, 120},
                                            {0.215722788, 125},
                                            {0.269129886, 130},
                                            {0.326725776, 135},
                                            {0.385368864, 140},
                                            {0.446106348, 145},
                                            {0.506843832, 150},
                                            {0.57072291, 155},
                                            {0.635649186, 160},
                                            {0.700575462, 165},
                                            {0.767596134, 170},
                                            {0.831475212, 175},
                                            {0.89535429, 180},
                                            {0.95818617, 185},
                                            {1.018923654, 190},
                                            {1.080708336, 195},
                                            {1.143540216, 200},
                                            {1.2042777, 205},
                                            {1.263967986, 210},
                                            {1.32470547, 215},
                                            {1.38230136, 220},
                                            {1.440944448, 225},
                                            {1.49749314, 230},
                                            {1.554041832, 235},
                                            {1.608496128, 240},
                                            {1.661903226, 245},
                                            {1.714263126, 250},
                                            {1.766623026, 255}};

void setup(void) {
    
  Serial.begin(9600);
  while (!Serial);  // wait for serial port to open!

  lcdInit();  

  // Serial.println("Orientation Sensor Test");
  // Serial.println("");  
 
  motorEncoderInit();
  imuInit();  
  robotPositionInit();

  orientationData  = imuOrientationData();
  // initPoseAngle = orientationData.orientation.x;
}

void loop(void) {
  static long int t1;
  static long int t2;  

  /* Capture starting time stamp */
  t1 = millis();
 
  lcdData dispData = {0};
  float incChangeOrientation = 0.0;  
  static float prevTheta;
  float angularVel_X = 0.0;
  int posCountsM1 = 0;
  float noOfRevM1 = 0.0;
  int posCountsM2 = 0;
  float noOfRevM2 = 0.0;
  static float prevPosX;
  static float prevPosY;
  robotPosition currentPos = {0};
  float currentOrientationTheta = 0.0;
  int currPulseCountM1 = 0;
  int currPulseCountM2 = 0;
  static int prevPulseCountM1;
  static int prevPulseCountM2;
  int setDirection = FORWARD_DIRECTION;
  refVelocity refVel = {0};
  currentRobotPosition currPosition = {0};
  targetRobotPosition desiredVals = {0};
  feedbackCntrlrTuneUpVals tuneUpVals = {0};
  wheelSpeedCommand whlSpd = {0};
  int leftMotorPWM = 0;        
  int rightMotorPWM = 0;
  int setDirectionMotorLeft = FORWARD_DIRECTION;
  int setDirectionMotorRight = FORWARD_DIRECTION;
          
  float **arr = (float **)malloc(ROW * sizeof(float *));

  for(int i = 0; i < ROW; i++){
    arr[i] = (float *)malloc(COLUMN * sizeof(float));
  }

  for(int i = 0; i < ROW; i++){
    for(int j = 0; j < COLUMN; j++){
      arr[i][j] = radPerSecToPWMTable[i][j];
    }
  }

  static int once = 1;
  String s =" ";

  while(once == 1){
    for(int i = 0; i < m; i++){
      s = (String)arr[i][0] + " " + (String)arr[i][1];
        Serial.println(s);
    }
    once = 2;
  }
   
  desiredVals.goalPos_x = 50.0;
  desiredVals.goalPos_y = 20.0;
  desiredVals.goalOri_theta = PI/2.0;

  tuneUpVals.k_rho = 0.5;
  tuneUpVals.k_alpha = 1.2;
  tuneUpVals.k_beta = -0.5;  
    
  /* Motor 1 encoder output reading */
  posCountsM1 = getPosCountEn1();
  noOfRevM1 = (float)posCountsM1 / (NO_OF_PULSES_PER_REV * MOTOR_GEAR_RATIO);
  currPulseCountM1 = posCountsM1 - prevPulseCountM1;
  Serial.println((float)noOfRevM1 / ((float)(t1)/60000));
   
  /* Motor 2 encoder output reading */
  posCountsM2 = getPosCountEn2();
  noOfRevM2 = (float)posCountsM2 / (NO_OF_PULSES_PER_REV * MOTOR_GEAR_RATIO);  
  currPulseCountM2 = posCountsM2 - prevPulseCountM2;
  // Serial.println(noOfRevM2);

  orientationData  = imuOrientationData();  
  angVelocityData = imuAngVelocityData();
  
  currentOrientationTheta = orientationData.orientation.x;

  if(currentOrientationTheta != 0.0){
    currentOrientationTheta = 360.00 - currentOrientationTheta;    
  }

  // Serial.println(prevPosX);  
  // Serial.println(prevPosY);  
  // Serial.println(prevPulseCountM1);
  // Serial.println(prevPulseCountM2);
  // Serial.println(posCountsM1);
  // Serial.println(posCountsM2);
  // Serial.println(currentOrientationTheta);
    
  currentPos = getRobotPosition(prevPosX, prevPosY, currPulseCountM1, currPulseCountM2, currentOrientationTheta); 

  currPosition.currPos_x = currentPos.pos_x;    
  currPosition.currPos_y = currentPos.pos_y;    
  currPosition.currOri_theta = currentOrientationTheta * PI / 180.0;  
  
  refVel = calcRefVelocity(currPosition, desiredVals, tuneUpVals);
  whlSpd = calWheelSpeed(refVel);         
  
  dispData.robotPos_X = currentPos.pos_x; 
  dispData.robotPos_Y = currentPos.pos_y;
  dispData.imuOrientiation_X = currentOrientationTheta;
  dispData.robotTargetPos_X = desiredVals.goalPos_x; 
  dispData.robotTargetPos_Y = desiredVals.goalPos_y;
  dispData.imuTargetOrientiation_X = desiredVals.goalOri_theta * 180.0/PI;
  
  // printLCDScreen(dispData);

  /* Display the floating point data */
  // Serial.print("Pose Angle: ");
  // Serial.print(orientationData .orientation.x, 4);
  // Serial.print("\tPosition X: ");
  // Serial.print(currentPos.pos_x, 4);
  // Serial.print("\tPosition Y: ");
  // Serial.print(currentPos.pos_y, 4);
  // Serial.println(" ");
  // Serial.print("Linear Vel: ");
  // Serial.print(refVel.LinVel, 4);
  // Serial.print("\tAngular Vel: ");
  // Serial.print(refVel.AngVel, 4);
  //  Serial.println(" ");
  // Serial.print("Left Wheel Speed: ");
  // Serial.print(whlSpd.wheelSpeedLeft, 4);
  // Serial.print("\tRight Wheel Speed: ");
  // Serial.print(whlSpd.wheelSpeedRight, 4);
  // Serial.println(" ");    

  if(whlSpd.wheelSpeedLeft > 255.0){
    leftMotorPWM = 255;    
  }
  else if(whlSpd.wheelSpeedLeft < 0){
    leftMotorPWM = (int)(0.0 - whlSpd.wheelSpeedLeft);
    setDirectionMotorLeft = -1;        
  }
  else{
    leftMotorPWM = (int)(whlSpd.wheelSpeedLeft);    
  }

  if(whlSpd.wheelSpeedRight > 255.0){
    rightMotorPWM = 255;    
  }
  else if(whlSpd.wheelSpeedRight < 0){
    rightMotorPWM = (int)(0.0 - whlSpd.wheelSpeedRight);
    setDirectionMotorRight = -1;    
  }
  else{
    rightMotorPWM = (int)(whlSpd.wheelSpeedRight);    
  }

  /* Both left and right motor PWM command is below 80%, we need to drive them faster
  as to avoid zero movement of the wheels */  
  if(leftMotorPWM <= MIN_SPEED && rightMotorPWM <= MIN_SPEED){
    leftMotorPWM += MIN_SPEED;
    rightMotorPWM += MIN_SPEED;   
  }  

  static int leftMotorPWM_tmp;
  static int rightMotorPWM_tmp;

  leftMotorPWM_tmp++;
  rightMotorPWM_tmp++;
  SATURATE(leftMotorPWM_tmp, 255);
  SATURATE(rightMotorPWM_tmp, 255);  
  Serial.println(leftMotorPWM_tmp);
  setMotor(FORWARD_DIRECTION, leftMotorPWM_tmp, PWM_OUT_M1, OUT1_M1, OUT2_M1); 
  setMotor(FORWARD_DIRECTION, leftMotorPWM_tmp, PWM_OUT_M2, OUT1_M2, OUT2_M2);

  float pwmOutput = interpolation2D(arr, m, n, 0.6);
  Serial.println("Result PWM = ");
  Serial.println(pwmOutput);
  
  prevPulseCountM1 = posCountsM1;
  prevPulseCountM2 = posCountsM2;
  prevTheta = currentOrientationTheta;
  prevPosX = currentPos.pos_x;
  prevPosY = currentPos.pos_y;

  delay(100);
  
  /* Capture ending time stamp */
  t2 = millis();
}