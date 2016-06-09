#ifndef _UNIVERSAL_PID_H_
#define _UNIVERSAL_PID_H_

#include <cmath>
#include <vector>
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "utility.h"

namespace containers = madara::knowledge::containers;

class PID
{
public:
  PID(containers::NativeDoubleVector & pid_coeffs_);
  ~PID();
  void set_t(std::chrono::time_point<std::chrono::high_resolution_clock> t_);
  double signal(double error, std::chrono::time_point<std::chrono::high_resolution_clock> t_);
  void reset(); // set integral accumulation term to 0

protected:
  double error_old;
  double error_derivative;
  double error_accumulator;
  containers::NativeDoubleVector pid_coeffs;
  double dt;  
  std::chrono::time_point<std::chrono::high_resolution_clock> t;
  std::chrono::time_point<std::chrono::high_resolution_clock> tOld;
};

#endif // _UNIVERSAL_PID_H_
