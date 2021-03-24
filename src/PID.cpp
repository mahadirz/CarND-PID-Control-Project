#include "PID.h"
#include <chrono>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  // Kp: The value for the proportional gain Kp
  Kp = Kp_;
  // Ki: The value for the integral gain Ki
  Ki = Ki_;
  // Kd: The value for the derivative gain Kd
  Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  // steering = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
  sumCte += cte;

   p_error = -Kp * cte;
   i_error = -Ki * sumCte ;
   d_error = -Kd * (cte - previousInput) ;

   previousInput = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  /*Compute PID Output*/
  return p_error + i_error + d_error;
}