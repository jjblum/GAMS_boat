#ifndef _LUTRA_ARTICULATED_FAN_DESIGN_H_
#define _LUTRA_ARTICULATED_FAN_DESIGN_H_

#include "design.h"

namespace designs
{
  class LutraArticulatedFan : public Design
  {
  public:
    LutraArticulatedFan();
    ~LutraArticulatedFan();  
    std::vector<double> motor_signals_from_effort_fractions(double thrust_fraction, double heading_fraction);    
    std::vector<double> thrust_and_moment_from_motor_signals(double m0, double m1);
  private:
  };
}

#endif // _LUTRA_ARTICULATED_FAN_DESIGN_H_
