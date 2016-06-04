#include "control.h"
#include "gams/loggers/GlobalLogger.h"


namespace knowledge = madara::knowledge;

// constructor
threads::control::control (Containers & containers_)
: containers(containers_)
{
  local_state.resize(STATE_DIMENSION);
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
    
    // pull the current state from the knowledgebase
    local_state = containers.local_state.to_record().to_doubles();
    
    // goal state - determined by containers for agent.id.source, agent.id.destination, and agent.id.desired_velocity
    // assume we are using LOS along a straight line all the time. PID used for heading, thrust effort set to a constant based on desired velocity
    // use the LOS_Line stuff from my python sim
    // Need a Design class and then child classes, again like my python sim
    // control thread has a design. It feeds the desired thrust and heading effort fractions to the design and the design provides the motor signals
}
