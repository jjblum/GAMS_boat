
#include "gams/loggers/GlobalLogger.h"
#include "compass_spoofer.h"

namespace knowledge = madara::knowledge;

// constructor
threads::compass_spoofer::compass_spoofer (threads::localization * localization_reference)
: LocalizationCaller(localization_reference)
{
}

// destructor
threads::compass_spoofer::~compass_spoofer ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::compass_spoofer::init (knowledge::KnowledgeBase & knowledge)
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
threads::compass_spoofer::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::compass_spoofer::run:" 
    " executing\n");

  std::vector<double> compass = {0.0 + utility::random_numbers::rand(-M_PI, M_PI)};
  Eigen::MatrixXd covariance(1, 1);
  covariance = Eigen::MatrixXd::Identity(1, 1); 
  Datum datum(SENSOR_TYPE::COMPASS, SENSOR_CATEGORY::LOCALIZATION, compass, covariance);
  new_sensor_callback(datum);  
}
