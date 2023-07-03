#include "pid.h"

PID::PID()
{
  resetPID();
}

void PID::runPID()
{
  P_part = Kp * error;
  I_part = Ki * Ts / 2 * (error + pre_error);
  D_part = Kd / Ts * (error - 2 * pre_error + pre_pre_error);
  output = P_part + I_part + D_part;
  pre_pre_error = pre_error;
  pre_error = error;

  output = (output > threshold) ? threshold : output;
  output = (output < -threshold) ? -threshold : output;
}

void PID::resetPID()
{
  error = 0;
  pre_error = 0;
  pre_pre_error = 0;
  output = 0;
}
