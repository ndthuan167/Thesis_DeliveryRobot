#ifndef PID_H
#define PID_H

class PID
{
public:
  PID();

  double Kp;
  double Ki;
  double Kd;
  double threshold;

  double P_part;
  double I_part;
  double D_part;
  double Ts;
  double error;
  double pre_error;
  double pre_pre_error;
  double output;

  void runPID();
  void resetPID();
};

#endif // PID_H
