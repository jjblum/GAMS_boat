#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <string>
#include <vector>

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




#endif // _SENSOR_H_
