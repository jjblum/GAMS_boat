#ifndef _LOCALIZATION_CALLER_H_
#define _LOCALIZATION_CALLER_H_

#include <functional>
#include "datum.h"
#include "threads/localization.h"

// a class to inherit that provides an easy way to establish the localization thread callback
class LocalizationCaller
{
public:
  LocalizationCaller(threads::localization * localization_reference);
  ~LocalizationCaller();
protected:
  std::function<void(Datum)> new_sensor_callback;
};


#endif // _LOCALIZATION_CALLER_H_
