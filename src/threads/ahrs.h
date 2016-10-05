
#ifndef   _THREAD_AHRS_H_
#define   _THREAD_AHRS_H_

#include <string>

#include "madara/threads/BaseThread.h"
#include "../boat_containers.h"
#include "AHRS.hpp"
#include "/home/pi/Navio2/C++/Navio/MPU9250.h"
#include "/home/pi/Navio2/C++/Navio/LSM9DS1.h"
#include "/home/pi/Navio2/C++/Navio/Util.h"
#include <unistd.h>
#include "localization.h"
#include "../utility.h"
#include "../datum.h"
#include "../localization_caller.h"

#include <cmath>
#include <functional>
#include <eigen3/Eigen/Core>




#define G_SI 9.80665
#define PI   3.14159

namespace threads
{
  /**
  * A custom thread generated by gams_sim_conf.pl
  **/
  class ahrs : public madara::threads::BaseThread, public LocalizationCaller
  {
  public:
    /**
     * Default constructor
     **/
    ahrs (Containers &, threads::localization * localization_reference);
    
    /**
     * Destructor
     **/
    virtual ~ahrs ();
    
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
    /// data plane if we want to access the knowledge base
    madara::knowledge::KnowledgeBase data_;
    Containers containers;
    // Objects

    InertialSensor *imu;
    AHRS    ahrs_;   // Mahony AHRS

    // Sensor data

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    // Orientation data

    float roll, pitch, yaw;

    // Timing data

    float offset[3];
    struct timeval tv;
    float dt, maxdt;
    float mindt = 0.01;
    unsigned long previoustime, currenttime;
    float dtsumm = 0;
    int isFirst = 1;
    void imuSetup();
  };
} // end namespace threads
	
#endif // _THREAD_AHRS_H_
