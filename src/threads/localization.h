
#ifndef   _THREAD_LOCALIZATION_H_
#define   _THREAD_LOCALIZATION_H_

#include <string>
#include <stdio.h>
#include <eigen3/Eigen/Core>

#include "madara/threads/BaseThread.h"
#include "../sensor.h"
#include "../boat_containers.h"

#define STATE_DIMENSION 6

typedef Eigen::Matrix< double, STATE_DIMENSION, 1> StateMatrix;

namespace threads
{
  /**
  * A custom thread generated by gams_sim_conf.pl
  **/
  class localization : public madara::threads::BaseThread
  {
  public:
    /**
     * Default constructor
     **/
    localization (Containers & containers_);
    
    /**
     * Destructor
     **/
    virtual ~localization ();
    
    /**
      * Initializes thread with MADARA context
      * @param   context   context for querying current program state
      **/
    virtual void init (madara::knowledge::KnowledgeBase & knowledge);

    /**
      * Executes the main thread logic
      **/
    virtual void run (void);

    void new_sensor_update(std::string sensor_name);

  private:
    /// data plane if we want to access the knowledge base
    madara::knowledge::KnowledgeBase data_;
    Containers containers;
    StateMatrix state;
  };
} // end namespace threads

#endif // _THREAD_LOCALIZATION_H_