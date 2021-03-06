
#include "gams/loggers/GlobalLogger.h"
#include "gps_spoofer.h"

namespace knowledge = madara::knowledge;

// constructor
threads::gps_spoofer::gps_spoofer (Containers containers_, threads::localization * localization_reference)
: containers(containers_), LocalizationCaller(localization_reference)
{
  t0 = utility::time_tools::now();
}

// destructor
threads::gps_spoofer::~gps_spoofer ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::gps_spoofer::init (knowledge::KnowledgeBase & knowledge)
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
threads::gps_spoofer::run (void)
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
    
  double lat = 40.4406 + 0.00001*utility::time_tools::dt(t0, utility::time_tools::now());
  double lon = -79.9959 + 0.00001*utility::time_tools::dt(t0, utility::time_tools::now());
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
  
  Eigen::MatrixXd covariance(2, 2);
  covariance = Eigen::MatrixXd::Identity(2, 2); 
  Datum datum(SENSOR_TYPE::GPS, SENSOR_CATEGORY::LOCALIZATION, gps_utm, covariance);
  new_sensor_callback(datum);
                    
}
