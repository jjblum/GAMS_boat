#ifndef _DESIGN_H_
#define _DESIGN_H_

#include <vector>

enum class DESIGN_TYPE
{
  LUTRA_TANK = 0,
  LUTRA_ARTICULATED_FAN = 1,
  LUTRA_TANK_FAN = 2,
  SERVAL_CATAMARAN = 3,
  FELIS_CATAMARAN = 4
};

namespace designs 
{
  class Design
  {
  public:
    Design();
    Design(DESIGN_TYPE type_);
    ~Design();
    virtual std::vector<double> motor_signals_from_effort_fractions(double thrust_fraction, double heading_fraction);
 
  protected:
    DESIGN_TYPE type;
  };
}

#endif // _DESIGN_H_
