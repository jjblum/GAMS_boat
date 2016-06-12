
#include "gams/loggers/GlobalLogger.h"
#include "ode_simulation_spoofer.h"

namespace knowledge = madara::knowledge;

// constructor
threads::ODE_sim_spoofer::ODE_sim_spoofer (Containers containers_, std::shared_ptr<designs::Design> design_, threads::localization * localization_reference)
: containers(containers_), design(design_), LocalizationCaller(localization_reference)
{
  t = utility::time_tools::now();
  t_double = 0.;
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
  printf("ode integration from t = %f  to t = %f\n", t_double, t_double + dt);
  
  t = tnew;
  t_double += dt;
  
                    
}

void threads::ODE_sim_spoofer::boat_ode (const state_array_type &x, state_array_type &dxdt, double t)
{
  // state: x y th xdot ydot thdot -- note that you don't have u explicitly! must get from xdot and ydot!
  //        0 1  2    3    4     5
  // xdot:  xdot ydot thdot xddot yddot thddot
  //           0    1     2     3     4      5
  double th = x[2];
  double u = x[3]*cos(th) + x[4]*sin(th);
  double w = -x[3]*sin(th) + x[4]*cos(th);  
  double udot = 1.0/design->mass()*(thrust_and_moment.at(0) - 0.5*WATER_KG_PER_SQ_M*design->drag_areas().at(0)*design->drag_coeffs().at(0)*std::abs(u)*u);
  double wdot = 1.0/design->mass()*(thrust_and_moment.at(1) - 0.5*WATER_KG_PER_SQ_M*design->drag_areas().at(1)*design->drag_coeffs().at(1)*std::abs(w)*w);
  dxdt[0] = x[3];
  dxdt[1] = x[4];
  dxdt[2] = x[5];
  dxdt[3] = udot*cos(th) - wdot*sin(th);
  dxdt[4] = udot*sin(th) + wdot*cos(th);
  dxdt[5] = 1.0/design->moment_of_inertia()*(thrust_and_moment.at(2) - 0.5*WATER_KG_PER_SQ_M*design->drag_areas().at(2)*design->drag_coeffs().at(2)*std::abs(x[5])*x[5]);
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
