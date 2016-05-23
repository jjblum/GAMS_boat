
#include "gams/loggers/GlobalLogger.h"
#include "localization.h"

namespace knowledge = madara::knowledge;

// constructor
threads::localization::localization (Containers & containers_)
: containers(containers_)
{
  state = StateMatrix::Zero();
}

// destructor
threads::localization::~localization ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::localization::init (knowledge::KnowledgeBase & knowledge)
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
threads::localization::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::localization::run:" 
    " executing\n");
}

void threads::localization::new_sensor_update(Datum datum)
{
  printf("Datum unique_id_count = %d    @ %s\n", Datum::unique_id_count, datum.get_human_readable_time().c_str());
  
}
