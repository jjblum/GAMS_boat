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
  private:
    std::vector<double> motor_signals_from_effort_fractions(double thrust_fraction, double heading_fraction);
  };
}

#endif // _LUTRA_ARTICULATED_FAN_DESIGN_H_
