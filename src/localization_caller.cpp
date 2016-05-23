#include "localization_caller.h"

LocalizationCaller::LocalizationCaller(threads::localization * localization_reference)
{
  new_sensor_callback = std::bind( & threads::localization::new_sensor_update, localization_reference, std::placeholders::_1);
}

LocalizationCaller::~LocalizationCaller()
{
}
