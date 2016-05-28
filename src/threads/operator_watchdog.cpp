
#include "gams/loggers/GlobalLogger.h"
#include "operator_watchdog.h"

namespace knowledge = madara::knowledge;

// constructor
threads::operator_watchdog::operator_watchdog (Containers & containers_)
: containers(containers_)
{
}

// destructor
threads::operator_watchdog::~operator_watchdog ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::operator_watchdog::init (knowledge::KnowledgeBase & knowledge)
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
threads::operator_watchdog::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::kb_print::run:" 
    " executing\n");

  if (!containers.heartbeat_operator.to_integer())
  {
    printf("WARNING: Operator is not sending packets to the boat!\n");
    // do stuff, like put the boat into teleop mode
    containers.teleop_status = 1;
  }
  containers.heartbeat_operator = 0;
}
