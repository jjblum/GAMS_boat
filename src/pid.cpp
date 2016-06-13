#include "pid.h"

PID::PID(containers::NativeDoubleVector & pid_coeffs_)
: pid_coeffs(pid_coeffs_)
{
  t = utility::time_tools::now();
}

PID::~PID()
{
}

void PID::set_t(std::chrono::time_point<std::chrono::high_resolution_clock> t_)
{
  t = t_;
}

double PID::signal(double error, std::chrono::time_point<std::chrono::high_resolution_clock> t_)
{  
  dt = utility::time_tools::dt(t, t_);
  t = t_;
  error_derivative = 0.;
  if (dt > 0.)
  {
    error_derivative = (error - error_old)/dt;
    if (std::abs(pid_coeffs[1]) > 0.)
    {
      error_accumulator += dt*error;
    }    
  }
  error_old = error;
  //printf("PID: P = %f  I = %f  D = %f\n", pid_coeffs[0], pid_coeffs[1], pid_coeffs[2]);
  //printf("PID: P term = %f   I term = %f   D term = %f\n", pid_coeffs[0]*error, pid_coeffs[1]*error_accumulator, pid_coeffs[2]*error_derivative);
  return pid_coeffs[0]*error + pid_coeffs[1]*error_accumulator + pid_coeffs[2]*error_derivative;
}

void PID::reset()
{
  error_accumulator = 0.;
}
