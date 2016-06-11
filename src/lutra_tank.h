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
  private:
    std::vector<double> motor_signals_from_effort_fractions(double thrust_fraction, double heading_fraction);
  };
}

#endif // _LUTRA_TANK_DESIGN_H_
