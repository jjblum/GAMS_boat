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
