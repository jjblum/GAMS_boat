#include "gams/loggers/GlobalLogger.h"
#include "ahrs.h"
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
namespace knowledge = madara::knowledge;

// constructor
threads::ahrs::ahrs (Containers & containers_, threads::localization * localization_reference)
: containers(containers_), LocalizationCaller(localization_reference)
{
}

// destructor
threads::ahrs::~ahrs ()
{
}

/**
 * Initialization to a knowledge base. If you don't actually need access
 * to the knowledge base, just scheduling things in madara::threads::Threader,
 * then you can decide to delete this function or simply do nothing inside of
 * the function.
 **/
void
threads::ahrs::init (knowledge::KnowledgeBase & knowledge)
{
  // point our data plane to the knowledge base initializing the thread
  data_ = knowledge;
  //imu = new MPU9250();
  imu = new LSM9DS1();
  if (!imu->probe()) {
      printf("Sensor not enabled\n");
      return;
  }
  imuSetup();
}

/**
 * Executes the actual thread logic. Best practice is to simply do one loop
 * iteration. If you want a long running thread that executes something
 * frequently, see the madara::threads::Threader::runHz method in your
 * controller.
 **/
void
threads::ahrs::run (void)
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
    
    //printf("{%6.2f, %6.2f, %6.2f} {%6.2f, %6.2f, %6.2f}, {%6.2f, %6.2f, %6.2f}\n", ax,ay,az,gx,gy,gz,mx,my,mz);
    ahrs_.AHRSupdate(ax, ay, az, gx, gy, gz, my, mx, -mz, dt);
    

    //------------------------ Read Euler angles ------------------------------

    ahrs_.getEuler(&roll, &pitch, &yaw);
    double boat_yaw = yaw;
    //------------------- Discard the time of the first cycle -----------------

    if (!isFirst)
    {
    	if (dt > maxdt) maxdt = dt;
    	if (dt < mindt) mindt = dt;
    }
    isFirst = 0;

    //------------- Console and network output with a lowered rate ------------

    dtsumm += dt;
    if(dtsumm > 0.05 && !isnan(boat_yaw))
    {
	//printf("dtsumm is %f\n", dtsumm);
        // Console output
        //printf("{%6.2f, %6.2f, %6.2f} {%6.2f, %6.2f, %6.2f}, {%6.2f, %6.2f, %6.2f}\n", ax,ay,az,gx,gy,gz,mx,my,mz);
	
	yaw += headingCorrection;
	if (yaw > 180.0)
	  yaw -= 360.0;
	if (yaw < -180.0)
	  yaw += 360; 
        /*yaw = (-euler_yaw - 90.0);
        if (yaw < -180.0)
        {
          yaw += 360.0;
        }*/
	boat_yaw = yaw;
        printf("ROLL: %+05.2f PITCH: %+05.2f YAW: %+05.2f PERIOD %.4fs RATE %dHz \n", roll, pitch, yaw * -1, dt, int(1/dt));
	//Convert to boat frame: Navio pitch is boat yaw
        boat_yaw = boat_yaw*M_PI/180.0;
        std::vector<double> compass = {boat_yaw};
        Eigen::MatrixXd covariance(1, 1);
        covariance = 0.00001*Eigen::MatrixXd::Identity(1, 1);
        Datum datum(SENSOR_TYPE::COMPASS, SENSOR_CATEGORY::LOCALIZATION, compass, covariance);
        new_sensor_callback(datum);

        dtsumm = 0;
    }
 
}

void threads::ahrs::imuSetup()
{
    //----------------------- MPU initialization ------------------------------

    imu->initialize();

    //-------------------------------------------------------------------------

	printf("Beginning Gyro calibration...\n");
	for(int i = 0; i<200; i++)
	{
	   imu->update();
           imu->read_gyroscope(&gx, &gy, &gz);

           gx *= 180 / PI;
           gy *= 180 / PI;
           gz *= 180 / PI;

	   offset[0] += (gx*0.0175);
       offset[1] += (gy*0.0175);
       offset[2] += (gz*0.0175);
	   usleep(10000);
	}
	offset[0]/=200.0;
	offset[1]/=200.0;
	offset[2]/=200.0;

	printf("Gyro Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
	//ahrs_.setGyroOffset(offset[0], offset[1], offset[2]);
/*
    printf("Starting Magnetometer calibration... Start spinning!\n");
    sleep(3);
    for(int i = 0; i < 2000; i++){
        imu->update();
        imu->read_magnetometer(&mx, &my, &mz);

        double magTemp[] = {mx, my, mz};
        
        for (int j = 0; j < 3; j++){
            if (magTemp[j] > magMax[j]){
                magMax[j] = magTemp[j];
            }
            if (magTemp[j] < magMin[j]){
                magMin[j] = magTemp[j];
            }
        }

        usleep(10000);
    }

    printf("Done. You can stop spinning now...\n");

        hardOffsets[0] = (magMin[0] + magMax[0])/2.0;
        hardOffsets[1] = (magMin[1] + magMax[1])/2.0;
        hardOffsets[2] = (magMin[2] + magMax[2])/2.0;

	printf("Mag Offsets are: %f %f %f\n", hardOffsets[0], hardOffsets[1], hardOffsets[2]);

      usleep(1000000);*/
	std::string line;
	std::ifstream infile("/home/pi/GAMS_boat/mag_params.txt");
	if (std::getline(infile, line) )
	{
  		std::istringstream iss(line);
		iss >> hardOffsets[0] >> hardOffsets[1] >>  hardOffsets[2] >> headingCorrection;
	}
}

