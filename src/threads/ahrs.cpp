
#include "gams/loggers/GlobalLogger.h"
#include "ahrs.h"

namespace knowledge = madara::knowledge;

// constructor
threads::AHRS::AHRS (threads::localization * localization_reference)
: LocalizationCaller(localization_reference)
{
  t = utility::time_tools::now();
}

// destructor
threads::AHRS::~AHRS ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::AHRS::init (knowledge::KnowledgeBase & knowledge)
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
threads::AHRS::run (void)
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

  /*
  std::vector<double> compass = {utility::angle_tools::wrap_to_pi(2.0*M_PI/10.0*
                                 utility::time_tools::dt(t0, utility::time_tools::now()))}; 
  Eigen::MatrixXd covariance(1, 1);
  covariance = 0.25*Eigen::MatrixXd::Identity(1, 1); 
  Datum datum(SENSOR_TYPE::COMPASS, SENSOR_CATEGORY::LOCALIZATION, compass, covariance);
  new_sensor_callback(datum);  
  */
}
