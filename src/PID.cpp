#include "PID.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

using namespace std;

/*
 * PID CONTROLLER FOR SIMULATOR USING REAL-TIME TWIDDLE UPDATES
 */

PID::~PID() {}

PID::PID(double kp_, double ki_, double kd_) {
  // Initial controller values
  p[0] = kp_;
  p[1] = ki_;
  p[2] = kd_;
  best_p[0] = p[0];
  best_p[1] = p[1];
  best_p[2] = p[2];

  dp[0] = 0.1 * p[0];
  dp[1] = 0.1 * p[1];
  dp[2] = 0.1 * p[2];

  dp_incremented[0] = false;
  dp_incremented[1] = false;
  dp_incremented[2] = false;
  dp_decremented[0] = false;
  dp_decremented[1] = false;
  dp_decremented[2] = false;

  sum_dp = 0;

  // Error values
  prev_cte = 0;
  avg_error = 0;
  p_error = 0;
  d_error = 0;
  i_error = 0;
  output_cmd = 0.0;
  previous_output_cmd = 0.0;

  // Twiddle values
  dp_threshold = 0.01;
  twiddle_update = 100;
  best_error = std::numeric_limits<double>::max();
  iter = 0;
  local_iter = 0;
  param = 0;
}

double PID::getOutputCommand(double cte) {
  // Error Updates
  d_error = cte - prev_cte;
  i_error += cte;
  output_cmd = -p[0] * cte - p[1] * i_error - p[2] * d_error;

  // Store the cte in prev_cte for derivative term
  prev_cte = cte;
  return output_cmd;
}

void PID::Twiddle() {
  sum_dp = 0.0;

  // Sum the P,I,D controller parameter changes
  for (int i = 0; i < 3; i++) {
    sum_dp += dp[i];
  }
  if (sum_dp > dp_threshold) {
    if (!dp_incremented[param] and !dp_decremented[param]) {
      p[param] += dp[param];
      dp_incremented[param] = true;
    }

    if (dp_incremented[param] and !dp_decremented[param]) {
      if (avg_error < best_error) {
        best_error = avg_error;
        best_p[param] = p[param];
        dp[param] *= 1.1;
        p[param] += dp[param];
        dp_incremented[param] = true;
      } else {
        p[param] = best_p[param] - dp[param];
        dp_incremented[param] = false;
        dp_decremented[param] = true;
        dp[param] *= 0.9;
      }
    }
    if (!dp_incremented[param] and dp_decremented[param]) {
      if (avg_error < best_error) {
        best_error = avg_error;
        best_p[param] = p[param];
        dp[param] *= 1.1;
        p[param] -= dp[param];
        dp_decremented[param] = true;
      } else {
        p[param] = best_p[param] + dp[param];
        dp_incremented[param] = true;
        dp_decremented[param] = false;
        dp[param] *= 0.9;
      }
    }
  }
  avg_error = 0;
  local_iter = 0;
}

double PID::twiddle(double cte) {
  //  using namespace std;

  iter += 1;
  local_iter += 1;
  avg_error = ((avg_error * (local_iter - 1)) + (cte * cte)) / local_iter;
  // Activate twiddle

  if (local_iter == twiddle_update) {
    std::cout << "kp : " << p[0] << " , ki : " << p[1] << " , kd : " << p[2]
              << "  "
              << "Avg Error: " << avg_error << " , Best Error : " << best_error
              << std::endl;
  }
  if (iter == twiddle_update) {
    best_error = avg_error;
  }

  // Only update PID parameters every twidle_update timesteps
  if (iter >= twiddle_update and iter % twiddle_update == 0) {
    Twiddle();
    ++param;
    if (param > 2) {
      param = 0;
    }
  }

  return avg_error;
}

void PID::getCurrentOutput(double cmd_) { previous_output_cmd = cmd_; }
