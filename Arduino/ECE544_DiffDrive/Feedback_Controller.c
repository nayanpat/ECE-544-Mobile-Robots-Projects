#include "Feedback_Controller.h"

extern refVelocity calcRefVelocity(currentRobotPosition currentVals, targetRobotPosition desiredVals, feedbackCntrlrTuneUpVals tuneUpVals){

  float rho, alpha, beta;
  refVelocity refVel;

  /* Calculate the polar coordinates */
  rho = sqrt(pow((desiredVals.goalPos_x - currentVals.currPos_x), 2) + pow((desiredVals.goalPos_y - currentVals.currPos_y), 2));
  alpha = -currentVals.currOri_theta + atan2((desiredVals.goalPos_y - currentVals.currPos_y), (desiredVals.goalPos_x - currentVals.currPos_x));
  beta = -currentVals.currOri_theta - alpha + desiredVals.goalOri_theta;

  /* Calculating the linear and angular velocity */
  refVel.LinVel = tuneUpVals.k_rho * rho;
  refVel.AngVel = tuneUpVals.k_alpha * alpha + tuneUpVals.k_beta * beta;
  refVel.rho = rho;
  refVel.alpha = alpha;
  refVel.beta = beta;

  return refVel;
}