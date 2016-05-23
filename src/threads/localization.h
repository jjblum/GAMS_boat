
#ifndef   _THREAD_LOCALIZATION_H_
#define   _THREAD_LOCALIZATION_H_

#include <string>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <queue>
#include <mutex>
#include <eigen3/Eigen/Core>

#include "madara/threads/BaseThread.h"
#include "../datum.h"
#include "../boat_containers.h"
#include "../utility.h"

#define STATE_DIMENSION 6
#define MAX_MAHALANOBIS_DIST 6.0
#define MAX_DATA_QUEUE_SIZE 100

typedef Eigen::Matrix< double, STATE_DIMENSION, 1> StateMatrix;
typedef Eigen::Matrix< double, STATE_DIMENSION, STATE_DIMENSION> StateSizedSquareMatrix; 

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

    void new_sensor_update(Datum datum); // public so it can be used as a callback
    void update();

  private:
    void predict();

    madara::knowledge::KnowledgeBase data_;
    Containers containers;
    std::queue<Datum> data_queue; // contains the datum objects that need to turn into sensor updates
    std::mutex queue_mutex;
    Datum current_datum;
    StateMatrix state;
    StateSizedSquareMatrix Q;
    StateSizedSquareMatrix P;
    StateSizedSquareMatrix G;
    StateSizedSquareMatrix Phi;
    StateSizedSquareMatrix Phi_k;
    Eigen::MatrixXd z;
    Eigen::MatrixXd R;
    Eigen::MatrixXd dz;
    Eigen::MatrixXd H;
    Eigen::MatrixXd S;
    std::chrono::time_point<std::chrono::high_resolution_clock> t; // current time
    
  };
} // end namespace threads

#endif // _THREAD_LOCALIZATION_H_
