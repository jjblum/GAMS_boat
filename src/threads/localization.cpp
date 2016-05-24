
#include "gams/loggers/GlobalLogger.h"
#include "localization.h"

namespace knowledge = madara::knowledge;

// constructor
threads::localization::localization (Containers & containers_)
: containers(containers_)
{
  state = StateMatrix::Zero(); // [x y th xdot ydot thdot]
  t = utility::time_tools::now();
  
  QBase = 0.1*StateSizedSquareMatrix::Identity();
  P = StateSizedSquareMatrix::Zero();
  P(0, 0) = 5.0;
  P(1, 1) = 5.0;
  Phi = StateSizedSquareMatrix::Zero();
  Phi_k = StateSizedSquareMatrix::Zero();
  G = StateSizedSquareMatrix::Identity();
}

// destructor
threads::localization::~localization ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::localization::init (knowledge::KnowledgeBase & knowledge)
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
threads::localization::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::localization::run:" 
    " executing\n");

  update();
}

void threads::localization::new_sensor_update(Datum datum)
{
  printf("Datum unique_id_count = %d    @ %s\n", Datum::unique_id_count, datum.human_readable_time().c_str());

  // Deal with initial GPS and compass values
  if (containers.localized == 0)
  {
    if (containers.gps_init == 0 && datum.type() == SENSOR_TYPE::GPS)
    {
      printf("Received first GPS: %.5f, %.5f\n", datum.value().at(0), datum.value().at(1)); 
      containers.gps_init = 1;
    }
    if (containers.compass_init == 0 && datum.type() == SENSOR_TYPE::COMPASS)
    {
      printf("Received first compass: %.4f\n", datum.value().at(0)*180.0/M_PI);
      containers.compass_init = 1;
    }
    if (containers.compass_init == 1 && containers.gps_init == 1)
    {
      containers.localized = 1;
    }
  }

  ///// BEGIN LOCKED SECTION
  std::lock_guard<std::mutex> lock(queue_mutex);
  if (data_queue.size() > MAX_DATA_QUEUE_SIZE)
  {
    printf("WARNING: localization queue is too long, throwing out oldest datum\n");
    data_queue.pop(); // throw out oldest item
  }
  data_queue.push(datum);// push Datum to queue
  printf("pushed datum to queue, size = %d\n", data_queue.size());
  ///// END LOCKED SECTION
}

void threads::localization::predict(double dt)
{
  // Kalman filter prediction step
  printf("prediction step: dt = %f\n", dt);
  
  Q = QBase*dt;
  
  Phi(0, 3) = dt;
  Phi(1, 4) = dt;
  Phi(2, 5) = dt;
  
  Phi_k(0, 3) = dt;
  Phi_k(1, 4) = dt;
  Phi_k(2, 5) = dt;
  
  state = Phi*state; // Phi*x
  // wrap theta
  while (std::abs(state(2,0)) > M_PI)
  {
    state(2,0) -= copysign(2.0*M_PI, state(2,0));
  }
  
  P = Phi_k*P*Phi_k.transpose() + G*Q*G.transpose();
  
}

void threads::localization::update()
{
  
  bool new_datum_available = false;
  ///// BEGIN LOCKED SECTION
  {
    // pop Datum from queue
    std::lock_guard<std::mutex> lock(queue_mutex);
    printf("is queue empty? size = %d\n", data_queue.size());
    if (!data_queue.empty())
    {
      new_datum_available = true;
      current_datum = data_queue.top();
      data_queue.pop();
      printf("popped datum from queue, size = %d\n", data_queue.size());      
    }
    else 
    {
      printf("no datum available\n");
    }
  }
  ///// END LOCKED SECTION

  //if there is a datum to process, continue, else return
  if (new_datum_available)
  {
    // prediction step
    // calculate dt using current time and datum time stamp    
    
    dt = std::chrono::duration_cast< std::chrono::duration<double> >(current_datum.timestamp()-t); // dt in seconds
    t = current_datum.timestamp();
    predict(dt.count());
    
    // convert value std::vector into a column matrix
    Eigen::Map<Eigen::MatrixXd> z_(current_datum.value().data(), current_datum.value().size(), 1);
    z = z_;
    R = current_datum.covariance();
    setH();
    
    
    
    printf("Processed a datum of type: %s\n", current_datum.type_string().c_str());
  }
}

void threads::localization::setH()
{
  double th = state(2, 0);
  double s = sin(th);
  double c = cos(th);
  
  H = Eigen::MatrixXd::Zero(z.rows(), STATE_DIMENSION);
  if (current_datum.type() == SENSOR_TYPE::GPS)
  {
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
  }
  else if (current_datum.type() == SENSOR_TYPE::COMPASS) 
  {
    H(0, 2) = 1.0;
  }
  else if (current_datum.type() == SENSOR_TYPE::GYRO) 
  {
    H(0, 5) = 1.0;
  }
  else if (current_datum.type() == SENSOR_TYPE::GPS_VELOCITY) 
  {
    H(0, 3) = 1.0;
    H(0, 4) = 1.0;    
  }
}
