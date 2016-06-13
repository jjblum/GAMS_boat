#include "lutra_tank.h"

designs::LutraTank::LutraTank()
: Design(DESIGN_TYPE::LUTRA_TANK)
{
  mass_ = 5.7833; // [kg]
  moment_of_inertia_ = 0.6; // [kg/m^2]
  max_thrust_per_motor_ = 12.5; // [N]
  drag_areas_ = {0.0108589939, 0.0424551192, 0.0424551192}; // surge, sway, rotation [m^2]
  drag_coeffs_ = {1.5, 1.088145891415693, 2.0};
  distance_between_motors = 0.3556;
}

designs::LutraTank::~LutraTank()
{
}

std::vector<double> designs::LutraTank::motor_signals_from_effort_fractions(double thrust_fraction, double heading_fraction)
{

  //printf("Design: t = %f   h = %f\n", thrust_fraction, heading_fraction);

  std::vector<double> result = {0., 0.};
  double m0 = 0;
  double m1 = 0;
  double motor_overage = 0;
  m0 = thrust_fraction + heading_fraction;
  m1 = thrust_fraction - heading_fraction;
  
  //printf("Design: motor signals BEFORE saturation correction:  m0 = %f   m1 = %f\n", m0, m1);
  
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
  
  //printf("Design: motor signals AFTER saturation correction:  m0 = %f   m1 = %f\n", m0, m1);
  printf("Design: equivalent effort fractions: thrust = %f   heading = %f\n", corrected_thrust_fraction, heading_fraction);
  
  result.at(0) = m0;
  result.at(1) = m1;
  return result;
}

std::vector<double> designs::LutraTank::thrust_and_moment_from_motor_signals(double m0, double m1)
{
  std::vector<double> result = {0., 0., 0.}; // surge thrust [N], sway thrust [N], moment [Nm]
  result.at(0) = (m0 + m1)/2.*(2.*max_thrust_per_motor_);
  result.at(1) = 0.;
  result.at(2) = -(m0 - m1)/2.*(2.*max_thrust_per_motor_*distance_between_motors);
  return result;
}


