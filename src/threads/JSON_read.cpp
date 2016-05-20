
#include "gams/loggers/GlobalLogger.h"
#include "JSON_read.h"

namespace knowledge = madara::knowledge;

// constructor
threads::JSON_read::JSON_read (std::shared_ptr<asio::serial_port> port_, Containers & containers_, threads::localization * localization_reference)
: threads::io_thread(port_, containers_)
{
  end_of_line_char = END_OF_LINE_CHAR;
  rejected_line_count = 0;
  new_sensor_callback = std::bind( & threads::localization::new_sensor_update, localization_reference, std::placeholders::_1);
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
              remove_quotes(type);
              
              if (type.compare("battery") == 0) 
              {
                std::string data = j.front().find("data").value();
                remove_quotes(data);
                std::vector<std::string> elems = split(data, ' ');
                float battery_voltage = std::stof(elems.at(0), nullptr);
                //printf("battery voltage = %.3f V\n", battery_voltage);
                containers.battery_voltage = battery_voltage;

                // TEST - try out callbacks with new sensors
                new_sensor_callback("battery");


              }
              // TODO - finish the other sensor parsing            
            }
            // ADAFRUIT GPS
            if (primary_key.substr(0,1) == "g") 
            {
              printf("received an Adafruit GPS reading\n");
              // TODO - finish the adafruit GPS parsing
            }            
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
