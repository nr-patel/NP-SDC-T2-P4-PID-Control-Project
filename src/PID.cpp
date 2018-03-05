#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

/* *********************************************************************** */
/*                              INITIALIZER                                */
/* *********************************************************************** */
/* It allows us to initialize the PID controller
   Inputs:
      - Kp: Proportional Gain
      - Ki: Integral Gain
      - Kd: Differential Gain
 */
void PID::Init(double Kp, double Ki, double Kd) {
  // 1) Assign the inputs values to the class objects
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
 
  // 2) Initialize the error to 0
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

/* *********************************************************************** */
/*                             UPDATE ERROR                                */
/* *********************************************************************** */
/* It allows us to update the error that we are achieving in order to change
   the behaviour of the controller.
 Inputs:
 - cte: the y position of the robot
 */
void PID::UpdateError(double cte) {
  i_error += cte;
  d_error = cte - p_error;
  p_error = cte;
}


/* *********************************************************************** */
/*                              TOTAL ERROR                                */
/* *********************************************************************** */
/* It allows us to calculate the total error of the controller.
 */
double PID::TotalError() {
  return - Kp * p_error - Ki * i_error - Kd * d_error;
}
