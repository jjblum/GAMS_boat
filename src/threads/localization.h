
#ifndef   _THREAD_LOCALIZATION_H_
#define   _THREAD_LOCALIZATION_H_

#include <string>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include <queue>
#include <mutex>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <GeographicLib/GeoCoords.hpp>

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
    void predict(double dt);
    void setH();

    madara::knowledge::KnowledgeBase data_;
    Containers containers;
    std::priority_queue<Datum, std::vector<Datum>, DatumComparison> data_queue;
    std::mutex queue_mutex;
    Datum current_datum;
    std::vector<double> eastingNorthingHeading;
    std::vector<double> location;
    StateMatrix state;
    StateSizedSquareMatrix Q; // growth of uncertainty with time (after time step modification)
    StateSizedSquareMatrix QBase; // growth of uncertainty with time (before time step modification)
    StateSizedSquareMatrix P;
    StateSizedSquareMatrix G;
    StateSizedSquareMatrix Phi;
    StateSizedSquareMatrix Phi_k;
    Eigen::MatrixXd z;
    Eigen::MatrixXd R;
    Eigen::MatrixXd dz;
    Eigen::MatrixXd H;
    Eigen::MatrixXd S;
    Eigen::MatrixXd K;
    std::chrono::time_point<std::chrono::high_resolution_clock> t; // current time
    double home_x;
    double home_y;    
    GeographicLib::GeoCoords coord;
  };
} // end namespace threads

#endif // _THREAD_LOCALIZATION_H_
