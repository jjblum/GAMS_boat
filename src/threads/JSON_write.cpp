
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
  motor_json["m0"] = { {"v", motor_signals.at(0)} };
  motor_json["m1"] = { {"v", motor_signals.at(1)} };
  //std::cout << std::setw(4) << motor_json << std::endl;
  raw_data.clear();
  raw_data = motor_json.dump(); // do NOT need an explicit newline character at the end
  //printf("%s\n", raw_data.c_str());
  strncpy(raw_buffer, raw_data.c_str(), raw_data.size());
  asio::error_code ec;
  port->write_some(asio::buffer(raw_buffer), ec);
  if (!ec) 
  {
      //printf("successful write\n");
      return;
  }
  else 
  {
      printf("ERROR: write_some() did not successfully write\n");
  }

}
