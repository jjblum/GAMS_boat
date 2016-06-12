#include "design.h"

designs::Design::Design()
{
}

designs::Design::Design(DESIGN_TYPE type_)
: type(type_)
{
}

designs::Design::~Design()
{
}

std::vector<double> designs::Design::motor_signals_from_effort_fractions(double thrust_fraction, double heading_fraction)
{
  std::vector<double> result = {0., 0.};
  return result;
}

std::vector<double> designs::Design::thrust_and_moment_from_motor_signals(double m0, double m1)
{
  std::vector<double> result = {0., 0., 0.};
  return result;
}

double designs::Design::mass()
{
  return mass_;
}

double designs::Design::moment_of_inertia()
{
  return moment_of_inertia_;
}

double designs::Design::max_thrust_per_motor()
{
  return max_thrust_per_motor_;
}

std::vector<double> designs::Design::drag_areas()
{
  return drag_areas_;
}

std::vector<double> designs::Design::drag_coeffs()
{
  return drag_coeffs_;
}
