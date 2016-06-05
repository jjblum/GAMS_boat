
#ifndef   _THREAD_AHRS_H_
#define   _THREAD_AHRS_H_

#include <string>
#include <cmath>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Core>

#include "madara/threads/BaseThread.h"

#include "localization.h"
#include "../datum.h"
#include "../utility.h"
#include "../localization_caller.h"
#include "../myahrs_plus.hpp"

#define AHRS_BAUD_RATE 115200
#define AHRS_PORT_NAME "/dev/tty_ahrs"
static const char* DIVIDER = "1";  // 100 Hz  (line taken from myahrs_plus_example.cpp)

namespace threads
{
  /**
  * A custom thread generated by gams_sim_conf.pl
  **/
  class AHRS : public madara::threads::BaseThread, public LocalizationCaller
  {
  public:
    /**
     * Default constructor
     **/
    AHRS (std::shared_ptr<WithRobot::MyAhrsPlus> ahrs_, threads::localization * localization_reference);
    
    /**
     * Destructor
     **/
    virtual ~AHRS ();
    
    /**
      * Initializes thread with MADARA context
      * @param   context   context for querying current program state
      **/
    virtual void init (madara::knowledge::KnowledgeBase & knowledge);

    /**
      * Executes the main thread logic
      **/
    virtual void run (void);

  private:
    madara::knowledge::KnowledgeBase data_;
    std::shared_ptr<WithRobot::MyAhrsPlus> ahrs;
    std::chrono::time_point<std::chrono::high_resolution_clock> t;
    uint32_t sample_count;
    WithRobot::SensorData sensor_data;
  };
} // end namespace threads

#endif // _THREAD_AHRS_H_
