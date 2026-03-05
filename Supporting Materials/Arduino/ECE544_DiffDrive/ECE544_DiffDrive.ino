#include "LCD.h"
#include "MotorEncoder.h"
#include "IMU.h"
#include "Position_Calc.h"
#include "Inverse_Kinematics.h"
#include "LookUp2D.h"

#define NO_OF_PULSES_PER_REV (float)11.0
#define MOTOR_GEAR_RATIO (float)47.5
#define FORWARD_DIRECTION (int)-1
#define REVERSE_DIRECTION (int)1
#define PI (float)(22.0 / 7.0)
#define MIN_SPEED (int)120
#define ROW (int)52
#define COLUMN (int)2
#define FLOAT_TO_INT(x) ((x) >= 0.0f ? (int)((x) + 0.5f) : (int)((x)-0.5f))

#define RHO_GAIN (float)0.3
#define ALPHA_GAIN (float)0.4
#define BETA_GAIN (float)-0.05

#define DESIRED_X (float)100.0
#define DESIRED_Y (float)80.0
#define DESIRED_POSE (float)(PI / 2.0)

sensors_event_t orientationData;
sensors_event_t angVelocityData;
float initPoseAngle = 45.0;

float radPerSecToPWMTable[ROW][COLUMN] = { { 0, 0 },
                                           { 0, 5 },
                                           { 0, 10 },
                                           { 0, 15 },
                                           { 0, 20 },
                                           { 0, 25 },
                                           { 0, 30 },
                                           { 0, 35 },
                                           { 0, 40 },
                                           { 0, 45 },
                                           { 0, 50 },
                                           { 0, 55 },
                                           { 0, 60 },
                                           { 0, 65 },
                                           { 0.001047198, 70 },
                                           { 0.001047198, 75 },
                                           { 0.001047198, 80 },
                                           { 0.001047198, 85 },
                                           { 0.001047198, 90 },
                                           { 0.00523599, 95 },
                                           { 0.01570797, 100 },
                                           { 0.161268492, 105 },
                                           { 0.082728642, 110 },
                                           { 0.122522166, 115 },
                                           { 0.165457284, 120 },
                                           { 0.215722788, 125 },
                                           { 0.269129886, 130 },
                                           { 0.326725776, 135 },
                                           { 0.385368864, 140 },
                                           { 0.446106348, 145 },
                                           { 0.506843832, 150 },
                                           { 0.57072291, 155 },
                                           { 0.635649186, 160 },
                                           { 0.700575462, 165 },
                                           { 0.767596134, 170 },
                                           { 0.831475212, 175 },
                                           { 0.89535429, 180 },
                                           { 0.95818617, 185 },
                                           { 1.018923654, 190 },
                                           { 1.080708336, 195 },
                                           { 1.143540216, 200 },
                                           { 1.2042777, 205 },
                                           { 1.263967986, 210 },
                                           { 1.32470547, 215 },
                                           { 1.38230136, 220 },
                                           { 1.440944448, 225 },
                                           { 1.49749314, 230 },
                                           { 1.554041832, 235 },
                                           { 1.608496128, 240 },
                                           { 1.661903226, 245 },
                                           { 1.714263126, 250 },
                                           { 1.766623026, 255 } };

float **arr = (float **)malloc(ROW * sizeof(float *));

void setup(void) {

  Serial.begin(9600);
  while (!Serial)
    delay(10);  // wait for serial port to open!

  lcdInit();

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  motorEncoderInit();
  imuInit();
  robotPositionInit();

  orientationData = imuOrientationData();
  // initPoseAngle = orientationData.orientation.x;

  for (int i = 0; i < ROW; i++) {
    arr[i] = (float *)malloc(COLUMN * sizeof(float));
  }

  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COLUMN; j++) {
      arr[i][j] = radPerSecToPWMTable[i][j];
    }
  }
}

void loop(void) {
  static long int t1 = 0;
  static long int t2 = 0;

  /* Capture starting time stamp */
  t1 = millis();

  // while (t1 < 3000) {
  //   setMotor(FORWARD_DIRECTION, 100, PWM_OUT_M1, OUT1_M1, OUT2_M1);
  //   setMotor(FORWARD_DIRECTION, 100, PWM_OUT_M2, OUT1_M2, OUT2_M2);
  //   t1 = millis();
  // }
  lcdData dispData;
  float incChangeOrientation = 0.0;
  static float prevTheta;
  float angularVel_X = 0.0;
  int currPosCountsM1 = 0;
  float noOfRevM1 = 0.0;
  int currPosCountsM2 = 0;
  float noOfRevM2 = 0.0;
  static float prevPosX;
  static float prevPosY;
  robotPosition currentPos = { 0 };
  float currentOrientationTheta = 0.0;
  int pulseCountIncrM1 = 0;
  int pulseCountIncrM2 = 0;
  static int prevPosCountM1;
  static int prevPosCountM2;
  refVelocity refVel = { 0 };
  currentRobotPosition currPosition = { 0 };
  targetRobotPosition desiredVals = { 0 };
  feedbackCntrlrTuneUpVals tuneUpVals = { 0 };
  wheelSpeedCommand whlSpd = { 0 };
  int leftMotorPWM = 0;
  int rightMotorPWM = 0;
  int setDirectionMotorLeft = FORWARD_DIRECTION;
  int setDirectionMotorRight = FORWARD_DIRECTION;
  static int prevSetDirectionMotorLeft;
  static int prevSetDirectionMotorRight;
  float incOrientation = 0.0;
  float relativeOrientation = 0.0;
  static float prevRelativeOrientation;
  static int oneTime = 1;

  desiredVals.goalPos_x = DESIRED_X;
  desiredVals.goalPos_y = DESIRED_Y;
  desiredVals.goalOri_theta = DESIRED_POSE;

  tuneUpVals.k_rho = RHO_GAIN;
  tuneUpVals.k_alpha = ALPHA_GAIN;
  tuneUpVals.k_beta = BETA_GAIN;

  /* Motor 1 encoder output reading */
  currPosCountsM1 = getPosCountEn1();
  noOfRevM1 = (float)currPosCountsM1 / (NO_OF_PULSES_PER_REV * MOTOR_GEAR_RATIO);
  pulseCountIncrM1 = currPosCountsM1 - prevPosCountM1;
  Serial.print("Pulse this cycle M1 = ");
  Serial.println(pulseCountIncrM1);

  /* Motor 2 encoder output reading */
  currPosCountsM2 = getPosCountEn2();
  noOfRevM2 = (float)currPosCountsM2 / (NO_OF_PULSES_PER_REV * MOTOR_GEAR_RATIO);
  pulseCountIncrM2 = currPosCountsM2 - prevPosCountM2;
  Serial.print("Pulse this cycle M1 = ");
  Serial.println(pulseCountIncrM2);

  orientationData = imuOrientationData();
  angVelocityData = imuAngVelocityData();

  currentOrientationTheta = orientationData.orientation.x;

  if (currentOrientationTheta == 0) {
    relativeOrientation = currentOrientationTheta;
  } else if (currentOrientationTheta <= 360.00 && currentOrientationTheta >= 180) {
    relativeOrientation = 360 - currentOrientationTheta;
  } else {
    relativeOrientation = 0 - currentOrientationTheta;
  }

  Serial.print("prevPosX = ");
  Serial.println(prevPosX);
  Serial.print("prevPosY = ");
  Serial.println(prevPosY);
  Serial.print("prevPosCountM1 =");
  Serial.println(prevPosCountM1);
  Serial.print("prevPosCountM2 =");
  Serial.println(prevPosCountM2);
  Serial.print("currPosCountsM1 =");
  Serial.println(currPosCountsM1);
  Serial.print("currPosCountsM2 = ");
  Serial.println(currPosCountsM2);
  Serial.print("currentOrientationTheta = ");
  Serial.println(currentOrientationTheta);

  currentPos = getRobotPosition(prevPosX, prevPosY, pulseCountIncrM1, pulseCountIncrM2, currentOrientationTheta);

  currPosition.currPos_x = currentPos.pos_x;
  currPosition.currPos_y = currentPos.pos_y;
  currPosition.currOri_theta = relativeOrientation * PI / 180.0;

  refVel = calcRefVelocity(currPosition, desiredVals, tuneUpVals);
  whlSpd = calWheelSpeed(refVel);

  dispData.robotPos_X = currentPos.pos_x;
  dispData.robotPos_Y = currentPos.pos_y;
  if(oneTime == 1){
    dispData.imuOrientiation_X = currentOrientationTheta;
  }
  else{
    dispData.imuOrientiation_X = 360.0 - currentOrientationTheta;
  }
  dispData.robotTargetPos_X = desiredVals.goalPos_x;
  dispData.robotTargetPos_Y = desiredVals.goalPos_y;
  dispData.imuTargetOrientiation_X = desiredVals.goalOri_theta * 180.0 / PI;

  printLCDScreen(dispData);

  // Wait for 5 sec to display and capture initial values of position and pose
  if(oneTime == 1){
    delay(5000);
    oneTime++;
  }

  /* Display the floating point data */
  Serial.print("Relative Angle: ");
  Serial.print(relativeOrientation);
  Serial.print("\tPosition X: ");
  Serial.print(currentPos.pos_x);
  Serial.print("\tPosition Y: ");
  Serial.print(currentPos.pos_y);
  Serial.println(" ");
  Serial.print("Linear Vel: ");
  Serial.print(refVel.LinVel);
  Serial.print("\tAngular Vel: ");
  Serial.print(refVel.AngVel);
  Serial.println(" ");
  Serial.print("rho: ");
  Serial.print(refVel.rho);
  Serial.print("\talpha: ");
  Serial.print(refVel.alpha);
  Serial.print("\tbeta: ");
  Serial.print(refVel.beta);
  Serial.println(" ");
  Serial.print("Left Wheel Speed: ");
  Serial.print(whlSpd.wheelSpeedLeft);
  Serial.print("\tRight Wheel Speed: ");
  Serial.print(whlSpd.wheelSpeedRight);
  Serial.println(" ");

  if (whlSpd.wheelSpeedLeft < 0) {
    setDirectionMotorLeft = REVERSE_DIRECTION;
  }
  if (whlSpd.wheelSpeedRight < 0) {
    setDirectionMotorRight = REVERSE_DIRECTION;
  }

  leftMotorPWM = interpolation2D(arr, ROW, COLUMN, fabs(whlSpd.wheelSpeedLeft));
  rightMotorPWM = interpolation2D(arr, ROW, COLUMN, fabs(whlSpd.wheelSpeedRight));

  Serial.print("Left Motor PWM Command: ");
  Serial.print(leftMotorPWM);
  Serial.print("\tRight Motor PWM Command: ");
  Serial.print(rightMotorPWM);
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");

  float currentPose = (360.0 - currentOrientationTheta);

  /* Stop robot within certain distance once reached goal position and pose 
  This is to avoid unncessary oscillations and unstable behavior */
  if ((currentPos.pos_x <= DESIRED_X + 2 && currentPos.pos_x >= DESIRED_X - 2) && 
      (currentPos.pos_y <= DESIRED_Y + 2 && currentPos.pos_y >= DESIRED_Y - 2) && 
      (currentPose <= DESIRED_POSE * 180/PI + 10 && currentPose >= DESIRED_POSE * 180/PI - 10)) {
    setMotor(setDirectionMotorLeft, 0, PWM_OUT_M1, OUT1_M1, OUT2_M1);
    setMotor(setDirectionMotorRight, 0, PWM_OUT_M2, OUT1_M2, OUT2_M2);
  }else{
    setMotor(setDirectionMotorLeft, leftMotorPWM, PWM_OUT_M1, OUT1_M1, OUT2_M1);
    setMotor(setDirectionMotorRight, rightMotorPWM, PWM_OUT_M2, OUT1_M2, OUT2_M2);
  }

  prevPosCountM1 = currPosCountsM1;
  prevPosCountM2 = currPosCountsM2;
  prevTheta = currentOrientationTheta;
  prevPosX = currentPos.pos_x;
  prevPosY = currentPos.pos_y;
  prevSetDirectionMotorLeft = setDirectionMotorLeft;
  prevSetDirectionMotorRight = setDirectionMotorRight;
  prevRelativeOrientation = currentOrientationTheta;

  /* Capture ending time stamp */
  t2 = millis();
}