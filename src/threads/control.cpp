#include "control.h"
#include "gams/loggers/GlobalLogger.h"


namespace knowledge = madara::knowledge;

// constructor
threads::control::control (Containers & containers_, std::shared_ptr<designs::Design> design_)
: containers(containers_), design(design_), heading_PID(containers_.LOS_surge_PID)
{  
  t = utility::time_tools::now();
  heading_PID.set_t(t);
}

// destructor
threads::control::~control ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::control::init (knowledge::KnowledgeBase & knowledge)
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
threads::control::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::control::run:" 
    " executing\n");        
    
    if (containers.teleop_status == 0)
    {        
      // goal state - determined by containers for agent.id.source, agent.id.destination, and agent.id.desired_velocity
      double x_dest, x_source, x_current, y_dest, y_source, y_current, th_full, th_current; 
      double dx_current, dx_full, dy_current, dy_full, L_current, L_full, dth;
      double projected_length, distance_from_ideal_line;
      double lookahead_distance, dx_lookahead, dy_lookahead;
      double heading_desired, heading_current, heading_error, heading_signal;
      x_dest = containers.self.agent.dest[0] - containers.self.agent.home[0];
      y_dest = containers.self.agent.dest[1] - containers.self.agent.home[1];
      x_source = containers.self.agent.source[0] - containers.self.agent.home[0];
      y_source = containers.self.agent.source[1] - containers.self.agent.home[1];
      x_current = containers.local_state[0];
      y_current = containers.local_state[1];
      heading_current = containers.local_state[2];
      dx_full = x_dest - x_source;
      dx_current = x_current - x_source;
      dy_full = y_dest - y_source;
      dy_current = y_current - y_source;
      L_full = sqrt(pow(dx_full, 2.) + pow(dy_full, 2.));
      L_current = sqrt(pow(dx_current, 2.) + pow(dy_current, 2.));
      th_full = atan2(dy_full, dx_full);
      th_current = atan2(dy_current, dx_current);
      dth = std::abs(th_full - th_current);
      projected_length = L_current*cos(dth);
      distance_from_ideal_line = L_current*sin(dth);
      lookahead_distance = containers.LOS_lookahead.to_double(); // TODO - use a dynamic lookahead?     
      std::vector<double> projected_state = {x_source + L_current*cos(th_full), y_source + L_current*sin(th_full)};
      std::vector<double> lookahead_state = {projected_state.at(0) + lookahead_distance*cos(th_full), projected_state.at(1) + lookahead_distance*sin(th_full)};
      // IMPORTANT NOTE: the ideal state (the lookahead) is allowed to go past the actual destination because it is just a tool to get the boat on top of the goal
      dx_lookahead = lookahead_state.at(0) - x_current;
      dy_lookahead = lookahead_state.at(1) - y_current;
      heading_desired = atan2(dy_lookahead, dx_lookahead);
      heading_error = utility::angle_tools::minimum_difference(heading_current - heading_desired); // fed into a 1 DOF PID for heading     
      t = utility::time_tools::now();
      heading_signal = heading_PID.signal(heading_error, t);
      std::vector<double> motor_signals = design->motor_signals_from_effort_fractions(containers.LOS_surge_effort_fraction.to_double(), heading_signal);
      containers.motor_signals.set(0, motor_signals.at(0));
      containers.motor_signals.set(1, motor_signals.at(1));      
      
      // TODO - set up velocity profile along the path that lets the desired thrust be modulated for slow start up and drift down. Independent of time, only depends on location along line.
      
    }
}
