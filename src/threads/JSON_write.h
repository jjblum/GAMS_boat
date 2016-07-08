
#ifndef   _THREAD_JSON_WRITE_H_
#define   _THREAD_JSON_WRITE_H_

#include "io_thread.h"
#include "../design.h"

namespace threads
{
  /**
  * A custom thread generated by gams_sim_conf.pl
  **/
  class JSON_write : public io_thread
  {
  public:
    /**
     * Default constructor
     **/
    JSON_write (std::shared_ptr<asio::serial_port> port_, Containers & containers_);
    
    /**
     * Destructor
     **/
    virtual ~JSON_write ();
    
    /**
      * Initializes thread with MADARA context
      * @param   context   context for querying current program state
      **/
    virtual void init (madara::knowledge::KnowledgeBase & knowledge);

    /**
      * Executes the main thread logic
      **/
    virtual void run (void);

    /**
      * Writes commands over serial
      */
    void write( json data );
  private:
  };
} // end namespace threads

#endif // _THREAD_JSON_WRITE_H_
