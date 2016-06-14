
#include "gams/loggers/GlobalLogger.h"
#include "ode_simulation_spoofer.h"

namespace knowledge = madara::knowledge;

// constructor
threads::ODE_sim_spoofer::ODE_sim_spoofer (Containers containers_, std::shared_ptr<designs::Design> design_, threads::localization * localization_reference)
: containers(containers_), design(design_), LocalizationCaller(localization_reference)
{
  t = utility::time_tools::now();
  t_double = 0.;
  
  _ODE_MASS = design->mass();
  _ODE_MOMENT_OF_INERTIA = design->moment_of_inertia();
  _ODE_DRAG_AREA_SURGE = design->drag_areas().at(0);
  _ODE_DRAG_AREA_SWAY = design->drag_areas().at(1);
  _ODE_DRAG_AREA_TURN = design->drag_areas().at(2);
  _ODE_DRAG_COEFF_SURGE = design->drag_coeffs().at(0);
  _ODE_DRAG_COEFF_SWAY = design->drag_coeffs().at(1);
  _ODE_DRAG_COEFF_TURN = design->drag_coeffs().at(2);  
  
  // initial GPS and compass
  std::vector<double> ahrs = {0.0, 0.0};
  Eigen::MatrixXd covariance_ahrs(2, 2); 
  covariance_ahrs = Eigen::MatrixXd::Identity(2, 2);
  covariance_ahrs(0, 0) = 10.0*M_PI/180.0;
  covariance_ahrs(1, 1) = 0.1;
  Datum datum_ahrs(SENSOR_TYPE::AHRS, SENSOR_CATEGORY::LOCALIZATION, ahrs, covariance_ahrs);
  new_sensor_callback(datum_ahrs);
  
  double lat = 40.4406;
  double lon = -79.9959;
  GeographicLib::GeoCoords coord(lat, lon);
  std::vector<double> gps_utm = {coord.Easting(), coord.Northing()};
  containers.gpsZone = coord.Zone();
  if (coord.Northp())
  {
    containers.northernHemisphere = 1;
  }
  else
  {
    containers.northernHemisphere = 0;
  }
  Eigen::MatrixXd covariance_gps(2, 2);
  covariance_gps = Eigen::MatrixXd::Identity(2, 2); 
  Datum datum_gps(SENSOR_TYPE::GPS, SENSOR_CATEGORY::LOCALIZATION, gps_utm, covariance_gps);
  new_sensor_callback(datum_gps);  
  
  t_to_gps = 0.;
  t_to_ahrs = 0.;
  
  state = containers.local_state.to_record().to_doubles();  // current state (used as initial state for ODE)
}

// destructor
threads::ODE_sim_spoofer::~ODE_sim_spoofer ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::ODE_sim_spoofer::init (knowledge::KnowledgeBase & knowledge)
{
  // point our data plane to the knowledge base initializing the thread
  data_ = knowledge;
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::ODE_sim_spoofer::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::gps_spoofer::run:" 
    " executing\n");
    
  std::chrono::time_point<std::chrono::high_resolution_clock> tnew = utility::time_tools::now();
  double dt = utility::time_tools::dt(t, tnew);
  motor_signals = containers.motor_signals.to_record().to_doubles();
  thrust_and_moment = design->thrust_and_moment_from_motor_signals(motor_signals.at(0), motor_signals.at(1));

  // apply odeint from t_double to t_double + dt  
  _ODE_THRUST_SURGE = thrust_and_moment.at(0);
  _ODE_THRUST_SWAY = thrust_and_moment.at(1);
  _ODE_MOMENT = thrust_and_moment.at(2);

  printf("ODE: integration from t = %f  to t = %f\n", t_double, t_double + dt);
  printf("    m0 = %f   m1 = %f\n", motor_signals.at(0), motor_signals.at(1));
  printf("    thrust = %f N   moment = %f Nm\n", thrust_and_moment.at(0), thrust_and_moment.at(2));
  std::cout << "    state before = " << state << std::endl;
  rk.do_step(boat_ode, state, t_double, dt);
  std::cout << "    state after = " << state << std::endl;  
  
  if (t_to_gps > 1./5.)
  {
    printf("ODE: new gps\n");
    std::vector<double> gps_utm = {state.at(0) + containers.self.agent.home[0], state.at(1) + containers.self.agent.home[1]};
    Eigen::MatrixXd covariance_gps(2, 2);
    covariance_gps = Eigen::MatrixXd::Identity(2, 2); 
    Datum datum_gps(SENSOR_TYPE::GPS, SENSOR_CATEGORY::LOCALIZATION, gps_utm, covariance_gps);
    new_sensor_callback(datum_gps);
    
    t_to_gps = 0.;
  }
  else
  {
    t_to_gps += dt;
  }
  
  if (t_to_ahrs > 1./20.)
  {
    printf("ODE: new ahrs\n");
    std::vector<double> ahrs = {state.at(2), state.at(5)};
    Eigen::MatrixXd covariance_ahrs(2, 2); 
    covariance_ahrs = Eigen::MatrixXd::Identity(2, 2);
    covariance_ahrs(0, 0) = 10.0*M_PI/180.0;
    covariance_ahrs(1, 1) = 0.1;
    Datum datum_ahrs(SENSOR_TYPE::AHRS, SENSOR_CATEGORY::LOCALIZATION, ahrs, covariance_ahrs);
    new_sensor_callback(datum_ahrs);
  
    t_to_ahrs = 0.;
  }
  else
  {
    t_to_ahrs += dt;
  }
  
  t = tnew;
  t_double += dt;
  
                    
}

void boat_ode (const std::vector<double> &x, std::vector<double> &dxdt, double t_)
{
  // state: x y th xdot ydot thdot -- note that you don't have u explicitly! must get from xdot and ydot!
  //        0 1  2    3    4     5
  // xdot:  xdot ydot thdot xddot yddot thddot
  //           0    1     2     3     4      5
  double th = x[2];
  double u = x[3]*cos(th) + x[4]*sin(th);
  double w = -x[3]*sin(th) + x[4]*cos(th);  
  double udot = 1.0/_ODE_MASS*(_ODE_THRUST_SURGE - 0.5*WATER_KG_PER_SQ_M*_ODE_DRAG_AREA_SURGE*_ODE_DRAG_COEFF_SURGE*std::abs(u)*u);
  double wdot = 1.0/_ODE_MASS*(_ODE_THRUST_SWAY - 0.5*WATER_KG_PER_SQ_M*_ODE_DRAG_AREA_SWAY*_ODE_DRAG_COEFF_SURGE*std::abs(w)*w);
  dxdt[0] = x[3];
  dxdt[1] = x[4];
  dxdt[2] = x[5];
  dxdt[3] = udot*cos(th) - wdot*sin(th);
  dxdt[4] = udot*sin(th) + wdot*cos(th);
  dxdt[5] = 1.0/_ODE_MOMENT_OF_INERTIA*(_ODE_MOMENT - 0.5*WATER_KG_PER_SQ_M*_ODE_DRAG_AREA_TURN*_ODE_DRAG_COEFF_TURN*std::abs(x[5])*x[5]); 
}

/*
def ode(state, t, boat):
    # derivative of state at input state and time
    # this is in Boat, not Design, because only the forces and moment are relevant
    rho = 1000.0  # density of water [kg/m^3]
    u = state[2]
    w = state[3]
    th = state[4]
    thdot = state[5]
    au = boat.design.dragAreas[0]
    aw = boat.design.dragAreas[1]
    ath = boat.design.dragAreas[2]
    cu = boat.design.dragCoeffs[0]
    cw = boat.design.dragCoeffs[1]
    cth = boat.design.dragCoeffs[2]
    qdot = np.zeros((6,))
    qdot[0] = u*math.cos(th) - w*math.sin(th)
    qdot[1] = u*math.sin(th) + w*math.cos(th)
    qdot[2] = 1.0/boat.design.mass*(boat.thrustSurge - 0.5*rho*au*cu*math.fabs(u)*u)
    qdot[3] = 1.0/boat.design.mass*(boat.thrustSway - 0.5*rho*aw*cw*math.fabs(w)*w)
    qdot[4] = thdot
    qdot[5] = 1.0/boat.design.momentOfInertia*(boat.moment - 0.5*rho*ath*cth*math.fabs(thdot)*thdot)

    # linear friction, only dominates when boat is moving slowly
    if u < 0.25:
        qdot[2] -= 1.0/boat.design.mass*5.0*u - np.sign(u)*0.001
    if w < 0.25:
        qdot[3] -= 1.0/boat.design.mass*5.0*w - np.sign(w)*0.001
    if thdot < math.pi/20.0:  # ten degrees per second
        qdot[5] -= 1.0/boat.design.momentOfInertia*5.0*thdot - np.sign(thdot)*0.001

    return qdot
*/
