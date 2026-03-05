#include "Feedback_Controller.h"

extern refVelocity calcRefVelocity(currentRobotPosition currentVals, targetRobotPosition desiredVals, feedbackCntrlrTuneUpVals tuneUpVals){

  float rho, alpha, beta;
  refVelocity refVel;

  /* Calculate the polar coordinates */
  rho = sqrt(sq(desiredVals.goalPos_x - currentVals.currPos_x) + sq(desiredVals.goalPos_y - currentVals.currPos_y));
  alpha = -currentVals.currOri_theta + atan2((desiredVals.goalPos_y - currentVals.currPos_y), (desiredVals.goalPos_x - currentVals.currPos_x));
  beta = -currentVals.currOri_theta - alpha + desiredVals.goalOri_theta;

  /* Calculating the linear and angular velocity */
  refVel.LinVel = tuneUpVals.k_rho * rho;
  refVel.AngVel = tuneUpVals.k_alpha * alpha + tuneUpVals.k_beta * beta;

  return refVel;
}