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
    virtual std::vector<double> thrust_and_moment_from_motor_signals(double m0, double m1);
    double mass();
    double moment_of_inertia();
    double max_thrust_per_motor();
    std::vector<double> drag_areas();
    std::vector<double> drag_coeffs();
 
  protected:
    DESIGN_TYPE type;
    double mass_;
    double moment_of_inertia_;
    double max_thrust_per_motor_;
    std::vector<double> drag_areas_;
    std::vector<double> drag_coeffs_;
  };
}

#endif // _DESIGN_H_

/*
self._mass = 0.0  # [kg]
self._momentOfInertia = 0.0  # [kg/m^2]
self._dragAreas = [0.0, 0.0, 0.0]  # surge, sway, rotation [m^2]
self._dragCoeffs = [0.0, 0.0, 0.0]  # surge, sway, rotation [-]
self._maxSpeed = 0.0  # [m/s]
self._minSpeed = 0.0
self._maxHeadingRate = 0.0  # maximum turning speed [rad/s]
self._maxForwardThrust = 0.0
self._speedVsMinRadius = np.zeros((1, 2))  # 2 column array, speed vs. min turning radius
*/
