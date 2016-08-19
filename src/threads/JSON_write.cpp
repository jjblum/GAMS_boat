
#include "gams/loggers/GlobalLogger.h"
#include "JSON_write.h"

namespace knowledge = madara::knowledge;

// constructor
threads::JSON_write::JSON_write (std::shared_ptr<asio::serial_port> port_, Containers & containers_)
: io_thread(port_, containers_)
{
}

// destructor
threads::JSON_write::~JSON_write ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::JSON_write::init (knowledge::KnowledgeBase & knowledge)
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
threads::JSON_write::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::JSON_write::run:" 
    " executing\n");

  // construct motor json
  json motor_json;
  std::vector<double> motor_signals = containers.motor_signals.to_record().to_doubles();
  if (motor_signals.size() < 2) 
  {
      printf("WARNING: there was not two motor signals in the knowledge base\n");
      return;
  }
  //printf("motor signal 0 = %.3f   motor signal 1 = %.3f\n", motor_signals.at(0), motor_signals.at(1));
  
  // TODO - finish the JSON motor signals for the various designs
  switch (containers.design_type.to_record().to_integer())
  {
    case (int)DESIGN_TYPE::LUTRA_TANK:
      motor_json["m0"] = { {"v", motor_signals.at(0)} };
      motor_json["m1"] = { {"v", motor_signals.at(1)} };
      //std::cout << "sending to motors:\n" <<  std::setw(4) << motor_json << std::endl;
      //std::cout << "sending to motors:\n" << motor_json << std::endl;
      break;
    case (int)DESIGN_TYPE::LUTRA_ARTICULATED_FAN:            
      motor_json["m0"] = { {"v", motor_signals.at(0)} };
      motor_json["s0"] = { {"p", motor_signals.at(1)} };      
      break;
    default:
      motor_json["m0"] = { {"v", motor_signals.at(0)} };
      motor_json["m1"] = { {"v", motor_signals.at(1)} };
  }

  write(motor_json);
  
  //Send arming signal to hardware
  if ( containers.arm_signal == 1 )
  {
    json arm_json;
    arm_json["o"] = { { "a", NULL } };
    containers.arm_signal = 0;
    write( arm_json );
  }
  
  //Check for error corde and inform eboard.
  if ( containers.error_signal != 0 )
  {
    json error_json;
    if ( containers.error_signal == 1)
    { 
      error_json["o"] = { { "e", "conn" } };
      write(error_json); 
    }
    else if ( containers.error_signal == 2 )
    { 
      error_json["o"] = { { "e", "gps"  } };
      write(error_json); 
    }
    else if ( containers.error_signal == 3 )
    { 
      error_json["o"] = { { "e", "ahrs" } };
      write(error_json); 
    }

  }
}
void threads::JSON_write::write(json & json_data)
{
  raw_data.clear();
  raw_data = json_data.dump();
  strncpy(raw_buffer, raw_data.c_str(), raw_data.size());
  raw_buffer[raw_data.size()] = '\n'; // explicitly add a newline character at the end?
  //printf("JSON write buffer: %s\n", raw_buffer);
  asio::error_code ec;
  port->write_some(asio::buffer(raw_buffer, raw_data.size()+1), ec);
  if (!ec) 
  {
      return;
  }
  else 
  {
      printf("ERROR: write_some() did not successfully write\n");
  }

}
