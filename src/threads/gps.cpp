
#include "gams/loggers/GlobalLogger.h"
#include "gps.h"

namespace knowledge = madara::knowledge;

// constructor
threads::gps::gps (Containers & containers, threads::localization * localization_reference)
: containers_(containers), LocalizationCaller(localization_reference)
{
}

// destructor
threads::gps::~gps ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::gps::init (knowledge::KnowledgeBase & knowledge)
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
threads::gps::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::gps::run:" 
    " executing\n");
    
    if (gps_.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
    { 
      //printf("Received Adafruit GPS lat/long = %f, %f\n", lat, lon);
      GeographicLib::GeoCoords coord(pos_data[1], pos_data[1]);
      std::vector<double> gps_utm = {coord.Easting(), coord.Northing()};
      containers_.gpsZone = coord.Zone();
      if (coord.Northp())
      {
        containers_.northernHemisphere = 1;
      }
      else
      {
        containers_.northernHemisphere = 0;
      }
      Eigen::MatrixXd covariance(2, 2);
      covariance = Eigen::MatrixXd::Identity(2, 2);
      Datum datum(SENSOR_TYPE::GPS, SENSOR_CATEGORY::LOCALIZATION, gps_utm, covariance);
      new_sensor_callback(datum);
    }
}
