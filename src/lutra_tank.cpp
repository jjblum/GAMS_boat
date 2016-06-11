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
  double m0 = 0;
  double m1 = 0;
  double motor_overage = 0;
  m0 = thrust_fraction + heading_fraction;
  m1 = thrust_fraction - heading_fraction;
  if (std::abs(m0) > 1.0)
  {
    motor_overage = copysign(std::abs(m0) - 1.0, m0);
  }
  if (std::abs(m1) > 1.0)
  {
    motor_overage = copysign(std::abs(m1) - 1.0, m1);
  }
  double corrected_thrust_fraction = thrust_fraction - motor_overage;
  m0 = corrected_thrust_fraction + heading_fraction;
  m1 = corrected_thrust_fraction - heading_fraction;
  result.at(0) = m0;
  result.at(1) = m1;
  return result;
}
