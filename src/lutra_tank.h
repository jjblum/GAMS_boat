#ifndef _LUTRA_TANK_DESIGN_H_
#define _LUTRA_TANK_DESIGN_H_

#include <cmath>
#include "design.h"

namespace designs
{
  class LutraTank : public Design
  {
  public:
    LutraTank();
    ~LutraTank();  
    std::vector<double> motor_signals_from_effort_fractions(double thrust_fraction, double heading_fraction);
    std::vector<double> thrust_and_moment_from_motor_signals(double m0, double m1);
  private:    
    double distance_between_motors;
  };
}

#endif // _LUTRA_TANK_DESIGN_H_
