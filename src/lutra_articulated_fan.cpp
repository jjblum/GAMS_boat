#include "lutra_articulated_fan.h"

designs::LutraArticulatedFan::LutraArticulatedFan()
: Design(DESIGN_TYPE::LUTRA_ARTICULATED_FAN)
{
}

designs::LutraArticulatedFan::~LutraArticulatedFan()
{
}

std::vector<double> designs::LutraArticulatedFan::motor_signals_from_effort_fractions(double thrust_fraction, double heading_fraction)
{
  std::vector<double> result = {0., 0.};
  return result;
}
