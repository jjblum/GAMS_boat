
#ifndef   _THREAD_GPS_SPOOFER_H_
#define   _THREAD_GPS_SPOOFER_H_

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>

#include "madara/threads/BaseThread.h"
#include "localization.h"
#include "../datum.h"
#include "../utility.h"
#include "../localization_caller.h"

namespace threads
{
  /**
  * A custom thread generated by gams_sim_conf.pl
  **/
  class gps_spoofer : public madara::threads::BaseThread, public LocalizationCaller
  {
  public:
    /**
     * Default constructor
     **/
    gps_spoofer (threads::localization * localization_reference);
    
    /**
     * Destructor
     **/
    virtual ~gps_spoofer ();
    
    /**
      * Initializes thread with MADARA context
      * @param   context   context for querying current program state
      **/
    virtual void init (madara::knowledge::KnowledgeBase & knowledge);

    /**
      * Executes the main thread logic
      **/
    virtual void run (void);

  private:
    /// data plane if we want to access the knowledge base
    madara::knowledge::KnowledgeBase data_;
  };
} // end namespace threads

#endif // _THREAD_GPS_SPOOFER_H_