
#ifndef   _THREAD_KB_PRINT_H_
#define   _THREAD_KB_PRINT_H_

#include <string>

#include "madara/threads/BaseThread.h"

namespace threads
{
  /**
  * A custom thread generated by gams_sim_conf.pl
  **/
  class kb_print : public madara::threads::BaseThread
  {
  public:
    /**
     * Default constructor
     **/
    kb_print ();
    
    /**
     * Destructor
     **/
    virtual ~kb_print ();
    
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

#endif // _THREAD_KB_PRINT_H_
