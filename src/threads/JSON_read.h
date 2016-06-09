
#ifndef   _THREAD_JSON_READ_H_
#define   _THREAD_JSON_READ_H_

#include <functional>
#include <eigen3/Eigen/Core>
#include <GeographicLib/GeoCoords.hpp>

#include "io_thread.h"
#include "localization.h"
#include "../utility.h"
#include "../datum.h"
#include "../localization_caller.h"

namespace threads
{
  /**
  * A custom thread generated by gams_sim_conf.pl
  **/
  class JSON_read : public io_thread, public LocalizationCaller
  {
  public:
    JSON_read (std::shared_ptr<asio::serial_port> port_, Containers & containers_, threads::localization * localization_reference);
    virtual ~JSON_read ();
    
    /**
      * Initializes thread with MADARA context
      * @param   context   context for querying current program state
      **/
    virtual void init (madara::knowledge::KnowledgeBase & knowledge);

    /**
      * Executes the main thread logic
      **/
    virtual void run (void);

    void callback_example (); // demonstrate c++11 callbacks

  private:
    char end_of_line_char;
    int rejected_line_count;
  };
} // end namespace threads

#endif // _THREAD_JSON_READ_H_
