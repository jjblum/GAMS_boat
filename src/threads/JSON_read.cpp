
#include "gams/loggers/GlobalLogger.h"
#include "JSON_read.h"

namespace knowledge = madara::knowledge;

// constructor
threads::JSON_read::JSON_read (std::shared_ptr<asio::serial_port> port_, std::shared_ptr<WithRobot::MyAhrsPlus> ahrs, Containers & containers_, threads::localization * localization_reference)
: threads::io_thread(port_, containers_), LocalizationCaller(localization_reference)
{
  end_of_line_char = END_OF_LINE_CHAR;
  rejected_line_count = 0;
}

// destructor
threads::JSON_read::~JSON_read ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::JSON_read::init (knowledge::KnowledgeBase & knowledge)
{
  // point our data plane to the knowledge base initializing the thread
  kb = knowledge;
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::JSON_read::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::JSON_read::run:" 
    " executing\n");

  asio::error_code ec;
  int bytes_read = port->read_some(asio::buffer(raw_buffer, BUFFER_SIZE), ec);
  if (!ec && bytes_read > 0) 
  {
    for (uint i = 0; i < bytes_read; i++) 
    {
      char c = raw_buffer[i];
      if (c == end_of_line_char) 
      {
        if (rejected_line_count > INITIAL_REJECT_COUNT) 
        {
          json j;
          try
          {
            j = json::parse(raw_data);
            std::string primary_key(j.begin().key().data());
            
            // SENSORS
            if (primary_key.substr(0,1) == "s") // look for a leading "s"
            {
              std::string type = j.front().find("type").value();
              utility::string_tools::remove_quotes(type);
              
              if (type.compare("battery") == 0) 
              {
                std::string data = j.front().find("data").value();
                utility::string_tools::remove_quotes(data);
                std::vector<std::string> elems = utility::string_tools::split(data, ' ');
                double battery_voltage = std::stod(elems.at(0), nullptr);
                //printf("battery voltage = %.3f V\n", battery_voltage);
                containers.battery_voltage = battery_voltage;
              }
              
              if (type.compare("AdafruitGPS") == 0)              
              {
                std::string nmea = j.front().find("data").value();
                utility::string_tools::remove_quotes(nmea);
                std::vector<std::string> elems = utility::string_tools::split(nmea, ',');

                //{"s1":{"type":"AdafruitGPS","data":"$GPRMC,183146.935,V,,,,,0.00,0.00,300616,,,N*49"}}
                double lat = -999.;
                double lon = -999.;
                if ( !elems.at((int)RMC_STRING::LAT_RAW).size()|| !elems.at((int)RMC_STRING::LON_RAW).size())
                {
                  printf("WARNING: Adafruit GPS does not have a fix\n");
                }
                else
                {                                 
                  lat = std::stod(elems.at((int)RMC_STRING::LAT_RAW), nullptr);
                  lon = std::stod(elems.at((int)RMC_STRING::LON_RAW), nullptr);
                  lat = GPRMC_to_degrees(lat)*( elems.at( (int)RMC_STRING::LAT_CARDINAL ).compare(NORTH) ?-1.:1.);
                  lon = GPRMC_to_degrees(lon)*(elems.at((int)RMC_STRING::LON_CARDINAL).compare(EAST)     ?-1.:1.);
                  //printf("Received Adafruit GPS lat/long = %f, %f\n", lat, lon);
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
              }
              else if (type.compare("AHRS") == 0)              
              {
                  //printf("Received AHRS data\n");
                  unsigned char * nmea_arr = new unsigned char[nmea.length()+1];
                  std::strcpy( (char *)nmea_arr, nmea.c_str() );
                  //Pass data into AHRS library, bypassing serial hardware interface
                  bool retVal = ahrs->feed( nmea_arr, nmea.length() );
                  
                  // TODO - handle ahrs error
                  if (!retVal)
                  {

                  }
              }
                    
                
              // TODO - finish the other sensor parsing            
            }
            // ADAFRUIT GPS
            /*
            if (primary_key.substr(0,1) == "g") // look for leading g
            {
              printf("received an Adafruit GPS reading\n");
              // TODO - finish the adafruit GPS parsing
              double lat = -999.;
              double lon = -999.;
              std::string lati = j.front().find("lati").value();
              std::string longi = j.front().find("longi").value();
              utility::string_tools::remove_quotes(lati);
              utility::string_tools::remove_quotes(longi);
              lat = std::stod(lati, nullptr);
              lon = std::stod(longi, nullptr);
              if (lat == -999. || lon == -999.)
              {
                printf("WARNING: Adafruit GPS did not return a correct latitude and longitude\n");
              }
              else
              {                                         
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
            }
            */            
          }
          catch (std::exception e) 
          {
              printf("ERROR: json parse failed: %s\n", e.what());
          }
        }
        else
        {
          rejected_line_count++;
        }
        raw_data.clear();
      }
      else
      {
        raw_data += c;
      }
    }        
  }
  else {
      printf("ERROR: port->read_some() error: %s\n", ec.message().c_str());
  }
}

double threads::JSON_read::GPRMC_to_degrees(double value)
{
  double fullDegrees = floor(value/100.0);
  double minutes = value - fullDegrees*100.0;
  return (fullDegrees + minutes/60.0);
}
