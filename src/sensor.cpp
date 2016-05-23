#include "sensor.h"

Sensor::Sensor()
{
}

Sensor::~Sensor()
{
}

Datum::Datum(SENSOR_TYPE type_, SENSOR_CATEGORY category_, std::vector<double> value_, Eigen::MatrixXd covariance_)
: type(type_), category(category_), value(value_), covariance(covariance_)
{
  unique_id = unique_id_count++;
  timestamp = std::chrono::high_resolution_clock::now();
  std::time_t now = std::chrono::high_resolution_clock::to_time_t(timestamp);
  human_readable_time = std::ctime(&now);
  if (type_ == SENSOR_TYPE::GPS)
  {
    type_string = "GPS";
  }
  else if (type_ == SENSOR_TYPE::COMPASS)
  {
    type_string = "COMPASS";
  }
  else if (type_ == SENSOR_TYPE::GYRO)
  {
    type_string = "GYRO";
  }
  else if (type_ == SENSOR_TYPE::ACCELEROMETER)
  {
    type_string = "ACCELEROMETER";
  }
  else if (type_ == SENSOR_TYPE::GPS_VELOCITY)
  {
    type_string = "GPS_VELOCITY";
  }
}

Datum::~Datum()
{
}

void Datum::set_location(GeographicLib::GeoCoords location_) 
{
  location = location_;
}

std::vector<double> Datum::get_value()
{
  return value;
}

Eigen::MatrixXd Datum::get_covariance()
{
  return covariance;
}

long Datum::get_unique_id()
{
  return unique_id;
}

std::string Datum::get_human_readable_time()
{
  return human_readable_time;
}

std::string Datum::get_type_string()
{
  return type_string;
}

long Datum::unique_id_count = -1;
