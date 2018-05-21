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
  const int p=0;
  const int i=1;
  const int d=2;
  double K[3];

  /*
  * Twiddle factors
  */
  double dK[3];
  double twiddleThreshold;
  bool   enableTwiddle;
  int    twiddleStep;
  int    twiddleInterval;
  double twiddleBestError;
  double twiddleTotalError;

  typedef enum {twiddleAdd, 
                twiddleSubtract, 
                twiddleAdjust} TwiddleState;
  TwiddleState twiddleState;

  int twiddleParam;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
