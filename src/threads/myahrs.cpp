#include "gams/loggers/GlobalLogger.h"
#include "myahrs.h"
#include "../utility.h"
#include <iostream>

namespace knowledge = madara::knowledge;

// constructor
threads::myahrs::myahrs (Containers & containers_, threads::localization * localization_reference)
: containers(containers_), LocalizationCaller(localization_reference)
{
}

// destructor
threads::myahrs::~myahrs ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::myahrs::init (knowledge::KnowledgeBase & knowledge)
{
  // point our data plane to the knowledge base initializing the thread
  data_ = knowledge;
  boost::asio::io_service io;
  this->port = std::make_shared<boost::asio::serial_port>(io);
  boost::system::error_code ec;
  bool portReady = false;
  while (!portReady){
      this->port->open("/dev/ttyAMA0", ec);
      if (!ec && this->port->is_open()){
          this->port->set_option(boost::asio::serial_port_base::baud_rate(115200));
          portReady=true;
          break;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  std::cout << "Port ready\n";
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::myahrs::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::ahrs::run:" 
    " executing\n");

    boost::system::error_code ec;
    int bytes_read = port->read_some(boost::asio::buffer(raw_buffer, 256), ec);
    if (!ec && bytes_read > 0)
    {
        for(int i = 0; i < bytes_read; i++)
        {
            char c = raw_buffer[i];
            if(c == '\n')
            {
                
		try{
			double yaw = std::stod( utility::string_tools::split(data, ',')[4] );
			yaw = -yaw + 90;
			if (yaw > 180)
			    yaw -= 360;
			yaw *= M_PI/180.0;
			std::vector<double> compass = {yaw};
			Eigen::MatrixXd covariance(1, 1);
			covariance = 0.00001*Eigen::MatrixXd::Identity(1, 1);
			Datum datum(SENSOR_TYPE::COMPASS, SENSOR_CATEGORY::LOCALIZATION, compass, covariance);
			new_sensor_callback(datum);
			data.erase();
			break;
		}catch(const std::invalid_argument &){
			std::cout << "AHRS invalid argument\n";
		}catch(const std::out_of_range &){
			std::cout << "AHRS invalid argument\n";
		}
            }
            else
                data += c;
        }

    }
}



