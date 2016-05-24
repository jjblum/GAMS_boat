
#include "gams/loggers/GlobalLogger.h"
#include "random_motor_signals.h"

namespace knowledge = madara::knowledge;

// constructor
threads::random_motor_signals::random_motor_signals (Containers & containers_)
: containers(containers_)
{
    //generator = std::make_shared<std::mt19937>(rd());
    //distribution = std::make_shared<std::uniform_real_distribution<>>(-1., 1.);
    //motor_signals.push_back(0.0);
    //motor_signals.push_back(0.0);
}

// destructor
threads::random_motor_signals::~random_motor_signals ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::random_motor_signals::init (knowledge::KnowledgeBase & knowledge)
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
threads::random_motor_signals::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::kb_print::run:" 
    " executing\n");

  //double motor1 = (*distribution)(*generator);
  //double motor2 = (*distribution)(*generator);
  //motor1 = round_to_decimal(motor1, 3);
  //motor2 = round_to_decimal(motor2, 3);
  ////printf("motor signal 1 = %.3f   motor signal 2 = %.3f\n", motor1, motor2);
  //motor_signals.at(0) = motor1;
  //motor_signals.at(1) = motor2;
  //containers.motor_signals.set(motor_signals);  
  containers.motor_signals.set(utility::random_numbers::rand(2, -1.0, 1.0));
}
