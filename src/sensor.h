#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <string>
#include <vector>
#include <chrono>
#include <ctime>
#include <stdio.h>
#include <GeographicLib/GeoCoords.hpp>
#include <eigen3/Eigen/Core>

enum class SENSOR_CATEGORY
{
  LOCALIZATION,
  ENVIRONMENTAL
};

enum class SENSOR_TYPE
{
  GPS,
  COMPASS,
  GYRO,
  ACCELEROMETER,
  GPS_VELOCITY,
  EC,
  TEMP,
  DO,
  PH,
  DEPTH,
  FLOW,
  WIFI_STRENGTH  
};


class Sensor
{
public:
  Sensor();
  ~Sensor();

protected:
  std::string name;
  SENSOR_TYPE type;
  SENSOR_CATEGORY category;
  double Hz;

private:
};


class Datum
{
public:
  Datum(SENSOR_TYPE type_, SENSOR_CATEGORY category_, std::vector<double> value_, Eigen::MatrixXd covariance_);
  ~Datum();
  void set_location(GeographicLib::GeoCoords location_);
  std::vector<double> get_value();
  Eigen::MatrixXd get_covariance();
  long get_unique_id();
  std::string get_human_readable_time();
  std::string get_type_string();

  static long unique_id_count;

private:
  SENSOR_TYPE type;
  SENSOR_CATEGORY category;
  long unique_id;
  std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
  std::string type_string;
  std::string human_readable_time;
  int dimension;
  std::vector<double> value;
  Eigen::MatrixXd covariance;
  GeographicLib::GeoCoords location;
};




#endif // _SENSOR_H_
