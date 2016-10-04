
#include "gams/loggers/GlobalLogger.h"
#include "dynamite.h"
#include "stdlib.h"
#include "stdlib.h"
namespace knowledge = madara::knowledge;

// constructor
threads::dynamite::dynamite (Containers & containers_)
: containers(containers_), pwrPin0(RPI_GPIO_17), pwrPin1(RPI_GPIO_18)
{
}

// destructor
threads::dynamite::~dynamite ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::dynamite::init (knowledge::KnowledgeBase & knowledge)
{
  // point our data plane to the knowledge base initializing the thread
  data_ = knowledge;
  if (!pwm.init(pwm_pins_[0]) || !pwm.init(pwm_pins_[1])) {
    fprintf(stderr, "Output Enable not set. Are you root?\n");
    return;
  }
/*  if (!pwm.init(pwr_pins_[0]) || !pwm.init(pwr_pins_[1])) {
    fprintf(stderr, "Output Enable not set. Are you root?\n");
    return;
  }*/
  pwm.enable(pwm_pins_[0]);
  pwm.set_period(pwm_pins_[0], 50);
  pwm.enable(pwm_pins_[1]);
  pwm.set_period(pwm_pins_[1], 50);
 
  /*pwm.enable(pwr_pins_[0]);
  pwm.set_period(pwr_pins_[0], 50);
  pwm.enable(pwr_pins_[1]);
  pwm.set_period(pwr_pins_[1], 50);
  */pwrPin0.init();
  pwrPin0.setMode(Navio::Pin::GpioMode::GpioModeOutput);
  pwrPin1.init();
  pwrPin1.setMode(Navio::Pin::GpioMode::GpioModeOutput);
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::dynamite::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::dynamite::run:" 
    " executing\n");
 
  //Arm motors if not already 
  if(!motors_armed)
  { 
    if(containers.arm_signal != 1)
      return;
    else
    {
      //arm motors
      printf("Arming\n");
      //pwm.set_duty_cycle(pwr_pins_[0], 0.1);
      //pwrPin0.write(0);
      //usleep(500000);
      pwm.set_duty_cycle(pwm_pins_[0], 1.5);
      pwm.set_duty_cycle(pwm_pins_[1], 1.5);
      //pwrPin0.write(1);
      //pwm.set_duty_cycle(pwr_pins_[0], 20.0);
      usleep(3000000);

      //pwm.set_duty_cycle(pwr_pins_[1], 0.1);
      //usleep(500000);
      //pwm.set_duty_cycle(pwm_pins_[1], 1.5);
      //pwm.set_duty_cycle(pwr_pins_[1], 20.0);
      //usleep(3000000);
		
      motors_armed = true;
    }
  }

  //Get motor signals from kb
  std::vector<double> motor_signals = containers.motor_signals.to_record().to_doubles();	

  setMotor(0,motor_signals[0]);   
  setMotor(1,motor_signals[1]);   
}

void
threads::dynamite::setMotor(int m_id, double v)
{
  //Clamp velocity signal
  if( v > 1.0)        v = 1.0;
  else if (v < -1.0)  v = -1.0;


  //Low pass filter the velocity signal
  if ( fabs(v - velocities_[m_id]) < VELOCITY_THRESHOLD )
    velocities_[m_id] = v;
  else
    velocities_[m_id] = (1-VELOCITY_ALPHA)*velocities_[m_id] + VELOCITY_ALPHA*v;


  //Convert motor signal to correct pulse width
  float command  = 1.5 + (float)velocities_[m_id]*0.5;
  pwm.set_duty_cycle(pwm_pins_[m_id], command);
}
