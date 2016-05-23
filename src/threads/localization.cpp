
#include "gams/loggers/GlobalLogger.h"
#include "localization.h"

namespace knowledge = madara::knowledge;

// constructor
threads::localization::localization (Containers & containers_)
: containers(containers_)
{
  state = StateMatrix::Zero();
  t = now();
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
  printf("Datum unique_id_count = %d    @ %s\n", Datum::unique_id_count, datum.human_readable_time().c_str());

  // Deal with initial GPS and compass values
  if (containers.localized == 0)
  {
    if (containers.gps_init == 0 && datum.type() == SENSOR_TYPE::GPS)
    {
      printf("Received first GPS: %.5f, %.5f\n", datum.value().at(0), datum.value().at(1)); 
      containers.gps_init = 1;
    }
    if (containers.compass_init == 0 && datum.type() == SENSOR_TYPE::COMPASS)
    {
      printf("Received first compass: %.4f\n", datum.value().at(0)*180.0/M_PI);
      containers.compass_init = 1;
    }
    if (containers.compass_init == 1 && containers.gps_init == 1)
    {
      containers.localized = 1;
    }
  }

  ///// BEGIN LOCKED SECTION
  // convert value std::vector into a column matrix
  Eigen::Map<Eigen::MatrixXd> z_(datum.value().data(), datum.value().size(), 1);
  z = z_;
  R = datum.covariance();
  ///// END LOCKED SECTION
  
  predict();
}

void threads::localization::predict()
{
  // Kalman filter prediction step
}
