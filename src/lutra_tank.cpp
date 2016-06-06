#include "lutra_tank.h"

designs::LutraTank::LutraTank()
: Design(DESIGN_TYPE::LUTRA_TANK)
{
}

designs::LutraTank::~LutraTank()
{
}

std::vector<double> designs::LutraTank::motor_signals_from_effort_fractions(double thrust_fraction, double heading_fraction)
{
  std::vector<double> result = {0., 0.};
  return result;
}
