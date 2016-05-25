
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
  P(2, 2) = 1.0;
  Phi = StateSizedSquareMatrix::Identity();
  Phi_k = StateSizedSquareMatrix::Identity();
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
  //printf("Datum unique_id_count = %d    @ %s\n", Datum::unique_id_count, datum.human_readable_time().c_str());

  // Deal with initial GPS and compass values
  if (containers.localized == 0)
  {
    if (containers.gps_init == 0 && datum.type() == SENSOR_TYPE::GPS)
    {
      printf("Received first GPS: %f, %f\n", datum.value().at(0), datum.value().at(1)); 
      containers.gps_init = 1;
      home_x = datum.value().at(0);
      home_y = datum.value().at(1);
      state(0, 0) = 0.0;
      state(1, 0) = 0.0;      
      std::cout << "Updated state = " << state.transpose() << std::endl;
    }
    if (containers.compass_init == 0 && datum.type() == SENSOR_TYPE::COMPASS)
    {
      printf("Received first compass: %f\n", datum.value().at(0));
      containers.compass_init = 1;
      state(2, 0) = datum.value().at(0);
      std::cout << "Updated state = " << state.transpose() << std::endl;
    }
    if (containers.compass_init == 1 && containers.gps_init == 1)
    {
      containers.localized = 1;
    }
    return; // don't do anything until at least 1 gps and compass measurement come through
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
  state(2,0) = utility::angle_tools::wrap_to_pi(state(2,0));
  
  P = Phi_k*P*Phi_k.transpose() + G*Q*G.transpose();
  
}

void threads::localization::update()
{
  
  bool new_datum_available = false;
  ///// BEGIN LOCKED SECTION
  {
    // pop Datum from queue
    std::lock_guard<std::mutex> lock(queue_mutex);
    //printf("is queue empty? size = %d\n", data_queue.size());
    if (!data_queue.empty())
    {
      new_datum_available = true;
      current_datum = data_queue.top();
      data_queue.pop();
      //printf("popped datum from queue, size = %d\n", data_queue.size());      
    }
    else 
    {
      //printf("no datum available\n");
    }
  }
  ///// END LOCKED SECTION

  //if there is a datum to process, continue, else return
  if (new_datum_available)
  {
    printf("Processing a datum of type: %s", current_datum.type_string().c_str());
    std::cout << " value = " << current_datum.value() << std::endl;
    // prediction step
    // calculate dt using current time and datum time stamp    
    
    //dt = std::chrono::duration_cast< std::chrono::duration<double> >(current_datum.timestamp()-t); // dt in seconds
    
    // TODO - what to do if dt happens to be negative
    
    //predict(dt.count());
    predict(utility::time_tools::dt(t, current_datum.timestamp()));
    t = current_datum.timestamp();
    
    // convert value std::vector into a column matrix
    std::vector<double> value = current_datum.value();
    if (current_datum.type() == SENSOR_TYPE::GPS)
    {
      value.at(0) -= home_x;
      value.at(1) -= home_y;
    }
    Eigen::Map<Eigen::MatrixXd> z_(value.data(), value.size(), 1);
    z = z_;
    R = current_datum.covariance();
    setH();
    
    K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
    
    dz = z - H*state; // TODO - alter the value for theta so that it isn't tricked by [-pi, pi] wrapped values
    if (current_datum.type() == SENSOR_TYPE::COMPASS)
    {
      dz(0, 0) = utility::angle_tools::minimum_difference(dz(0, 0)); // use true angular difference, not algebraic difference
    }
            
    S = H*P*H.transpose();
    
    if (S.determinant() < pow(10.0, -12.0))
    {
      printf("WARNING: innovation covariance is singular for sensor type: %s\n", current_datum.type_string().c_str());
      std::cout << "z = " << z << std::endl;
      std::cout << "H = " << H << std::endl;
      std::cout << "R = " << R << std::endl;
      std::cout << "P = " << P << std::endl;
      std::cout << "K = " << K << std::endl;
      std::cout << "S = " << S << std::endl;
      std::cout << "det(S) = " << S.determinant() << " vs. " << pow(10.0, -6.0) << std::endl;
      return;      
    }
    
    double d = sqrt((dz.transpose()*S.inverse()*dz)(0, 0));
    printf("Mahalonobis distance = %f\n", d);
    
    state += K*dz;
    P = (StateSizedSquareMatrix::Identity() - K*H)*P;        
    std::cout << "Updated state = " << state.transpose() << std::endl;
  }
}

void threads::localization::setH()
{
  //double th = state(2, 0);
  //double s = sin(th);
  //double c = cos(th);
  
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
    H(1, 4) = 1.0;    
  }
}
