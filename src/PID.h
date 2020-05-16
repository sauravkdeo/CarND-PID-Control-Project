#ifndef PID_H
#define PID_H

class PID {
 public:
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /*
   * Coefficients
   */
  double p[3];
  double best_p[3];
  double dp[3];
  //  double factor[3];
  bool dp_incremented[3];
  bool dp_decremented[3];
  /*
   * Error values
   */
  double prev_cte;
  double avg_error;
  double previous_output_cmd;
  double output_cmd;

  /*
   * Twiddle values
   */
  double best_error;
  double dp_threshold;
  int iter, local_iter;
  int twiddle_update;
  int param;

  double sum_dp;

  /*
   * Constructor
   */
  PID(double, double, double);

  /*
   * Destructor.
   */
  virtual ~PID();

  /*
   * Initialize PID.
   */

  /*
   * Update the PID error variables given cross track error.
   */
  double getOutputCommand(double);

  /*
   * Twiddle the PID values
   */
  void Twiddle();

  /*
   * Calculate the total PID error.
   */
  double twiddle(double);

  void getCurrentOutput(double);
};

#endif /* PID_H */
