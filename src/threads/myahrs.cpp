#include "gams/loggers/GlobalLogger.h"
#include "myahrs.h"
#include "../utility.h"

namespace knowledge = madara::knowledge;

// constructor
threads::myahrs::myahrs (Containers & containers_, threads::localization * localization_reference)
: containers(containers_), LocalizationCaller(localization_reference)
{
}

// destructor
threads::myahrs::~myahrs ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::myahrs::init (knowledge::KnowledgeBase & knowledge)
{
  // point our data plane to the knowledge base initializing the thread
  data_ = knowledge;
  asio::io_service io;
  this->port = std::make_shared<asio::serial_port>(io);
  asio::error_code ec;
  bool portReady = false;
  while (!portReady){
      this->port->open("\dev\ttyAMA0", ec);
      if (!ec && this->port->is_open()){
          this->port->set_option(asio::serial_port_base::baud_rate(115200));
          portReady=true;
          break;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));:
  }

}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::myahrs::run (void)
{
  /**
   * the MADARA logger is thread-safe, fast, and allows for specifying
   * various options like output files and multiple output targets (
   * e.g., std::cerr, a system log, and a thread_output.txt file). You
   * can create your own custom log levels or loggers as well.
   **/
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "threads::ahrs::run:" 
    " executing\n");
  
    //----------------------- Calculate delta time ----------------------------

	gettimeofday(&tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;
	if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
        gettimeofday(&tv,NULL);
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;

    //-------- Read raw measurements from the MPU and update AHRS --------------

    // Accel + gyro + mag.
    // Soft and hard iron calibration required for proper function.
    
    imu->update();
    imu->read_accelerometer(&ax, &ay, &az);
    imu->read_gyroscope(&gx, &gy, &gz);
    imu->read_magnetometer(&mx, &my, &mz);

    ax /= G_SI;
    ay /= G_SI;
    az /= G_SI;
    gx =  gx* 180.0*0.0175 / PI - offset[0];
    gy =  gy* 180.0*0.0175 / PI - offset[1];
    gz =  gz* 180.0*0.0175 / PI - offset[2];

    mx -= hardOffsets[0];
    my -= hardOffsets[1];
    mz -= hardOffsets[2];
    
    printf("{%6.2f, %6.2f, %6.2f} {%6.2f, %6.2f, %6.2f}, {%6.2f, %6.2f, %6.2f}\n", ax,ay,az,gx,gy,gz,mx,my,mz);
    ahrs_.MadgwickAHRSupdate(ax, ay, az, gx, gy, gz, my, mx, -mz, dt);
    

    //------------------------ Read Euler angles ------------------------------

    ahrs_.getEuler(&roll, &pitch, &yaw);

    //------------------- Discard the time of the first cycle -----------------

    if (!isFirst)
    {
    	if (dt > maxdt) maxdt = dt;
    	if (dt < mindt) mindt = dt;
    }
    isFirst = 0;

    //------------- Console and network output with a lowered rate ------------

    dtsumm += dt;
    if(dtsumm > 0.05)
    {
        // Console output
        printf("ROLL: %+05.2f PITCH: %+05.2f YAW: %+05.2f PERIOD %.4fs RATE %dHz \n", roll, pitch, yaw * -1, dt, int(1/dt));

        /*yaw = (-euler_yaw - 90.0);
        if (yaw < -180.0)
        {
          yaw += 360.0;
        }*/
        yaw *= M_PI/180.0;
        std::vector<double> compass = {yaw};
        Eigen::MatrixXd covariance(1, 1);
        covariance = 0.00001*Eigen::MatrixXd::Identity(1, 1);
        Datum datum(SENSOR_TYPE::COMPASS, SENSOR_CATEGORY::LOCALIZATION, compass, covariance);
        new_sensor_callback(datum);

        dtsumm = 0;
    }
 
}



