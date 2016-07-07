
#include "gams/loggers/GlobalLogger.h"
#include "ahrs.h"

namespace knowledge = madara::knowledge;

// constructor
               
threads::AHRS::AHRS (std::shared_ptr<WithRobot::MyAhrsPlus> ahrs_, threads::localization * localization_reference)
: ahrs(ahrs_), LocalizationCaller(localization_reference)
{
  t = utility::time_tools::now();
  
  /*if (ahrs->cmd_ascii_data_format("RPYIMU") == false) // sequence number, roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, magnet_y, magnet_y, magnet_z, temperature  
  {
    printf("WARNING: AHRS did not accept data format command\n");
  }
  
  if(ahrs->cmd_divider(DIVIDER) ==false) 
  {
    //handle_error("cmd_divider() returns false");
  }
  
  if(ahrs->cmd_mode("AT") ==false) 
  {
    //handle_error("cmd_mode() returns false");
  }*/  
  sample_count = 0;
}

// destructor
threads::AHRS::~AHRS ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::AHRS::init (knowledge::KnowledgeBase & knowledge)
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
threads::AHRS::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::compass_spoofer::run:" 
    " executing\n");

  ahrs->cmd_trigger(); // send trigger signal to myAHRS+
  if(ahrs->wait_data() == true) 
  {
    sample_count = ahrs->get_sample_count();
  
    // copy sensor data
    ahrs->get_data(sensor_data);

    // print data -- remember the MyAHRS+ uses degrees as its units
    WithRobot::EulerAngle& e = sensor_data.euler_angle;
    WithRobot::ImuData<float>& imu = sensor_data.imu;
    //printf("%04d) EulerAngle (roll = %.2f, pitch = %.2f, yaw = %.2f), Gyro (roll = %.2f, pitch = %.2f, yaw = %.2f), Accel (x = %.2f, y = %.2f, z = %.2f)\n", 
    //  sample_count, e.roll, e.pitch, e.yaw, imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az);
      
    // create ahrs (compass and gyro) Datum and send to localization
    //std::vector<double> compass_and_gyro = {(double)e.yaw*M_PI/180.0, (double)imu.gz*M_PI/180.0};
    //Eigen::MatrixXd covariance(2, 2); 
    //covariance = Eigen::MatrixXd::Identity(2, 2);
    //covariance(0, 0) = 0.0001; //10.0*M_PI/180.0;
    //covariance(1, 1) = 100.;
    //Datum datum(SENSOR_TYPE::AHRS, SENSOR_CATEGORY::LOCALIZATION, compass_and_gyro, covariance);
    //new_sensor_callback(datum);
    double yaw = ((double)-e.yaw + 90.0);
    if (yaw > 180.0)
    {
      yaw -= 360.0;
    }
    yaw *= M_PI/180.0;
    std::vector<double> compass = {yaw}; 
    Eigen::MatrixXd covariance(1, 1);
    covariance = 0.00001*Eigen::MatrixXd::Identity(1, 1); 
    Datum datum(SENSOR_TYPE::COMPASS, SENSOR_CATEGORY::LOCALIZATION, compass, covariance);
    new_sensor_callback(datum);
  }
}
