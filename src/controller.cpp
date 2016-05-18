

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/threads/Threader.h"
#include "gams/controllers/BaseController.h"
#include "gams/loggers/GlobalLogger.h"

// DO NOT DELETE THIS SECTION

// begin algorithm includes
// end algorithm includes

// begin platform includes
#include "platforms/boat.h"
// end platform includes

// begin thread includes
#include "threads/localization.h"
#include "threads/control.h"
#include "threads/analytics.h"
#include "threads/sensing.h"
#include "threads/JSON_read.h"
#include "threads/JSON_write.h"
#include "threads/kb_print.h"
// end thread includes

// begin transport includes
// end transport includes

// begin other includes
#include "boat_containers.h"
#include "asio/io_service.hpp"
#include "asio/error_code.hpp"
#include "asio/serial_port.hpp"
#include <memory>
// end other includes

// END DO NOT DELETE THIS SECTION

const std::string default_broadcast ("192.168.1.255:15000");
// default transport settings
std::string host ("");
const std::string default_multicast ("239.255.0.1:4150");
madara::transport::QoSTransportSettings settings;

// create shortcuts to MADARA classes and namespaces
namespace controllers = gams::controllers;
typedef madara::knowledge::KnowledgeRecord   Record;
typedef Record::Integer Integer;

const std::string KNOWLEDGE_BASE_PLATFORM_KEY (".platform");
bool plat_set = false;
std::string platform ("boat");
std::string algorithm ("debug");
std::vector <std::string> accents;

// controller variables
double period (1.0);
double loop_time (50.0);

// madara commands from a file
std::string madara_commands = "";

// number of agents in the swarm
Integer num_agents (-1);

// file path to save received files to
std::string file_path;

void print_usage (char * prog_name)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
"\nProgram summary for %s:\n\n" 
"     Loop controller setup for gams\n" 
" [-A |--algorithm type]        algorithm to start with\n" 
" [-a |--accent type]           accent algorithm to start with\n" 
" [-b |--broadcast ip:port]     the broadcast ip to send and listen to\n" 
" [-d |--domain domain]         the knowledge domain to send and listen to\n" 
" [-e |--rebroadcasts num]      number of hops for rebroadcasting messages\n" 
" [-f |--logfile file]          log to a file\n" 
" [-i |--id id]                 the id of this agent (should be non-negative)\n" 
" [--madara-level level]        the MADARA logger level (0+, higher is higher detail)\n" 
" [--gams-level level]          the GAMS logger level (0+, higher is higher detail)\n" 
" [-L |--loop-time time]        time to execute loop\n"
" [-m |--multicast ip:port]     the multicast ip to send and listen to\n" 
" [-M |--madara-file <file>]    file containing madara commands to execute\n" 
"                               multiple space-delimited files can be used\n" 
" [-n |--num_agents <number>]   the number of agents in the swarm\n" 
" [-o |--host hostname]         the hostname of this process (def:localhost)\n" 
" [-p |--platform type]         platform for loop (vrep, dronerk)\n" 
" [-P |--period period]         time, in seconds, between control loop executions\n" 
" [-q |--queue-length length]   length of transport queue in bytes\n" 
" [-r |--reduced]               use the reduced message header\n" 
" [-t |--target path]           file system location to save received files (NYI)\n" 
" [-u |--udp ip:port]           a udp ip to send to (first is self to bind to)\n" 
"\n",
        prog_name);
  exit (0);
}

// handle command line arguments
void handle_arguments (int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1 (argv[i]);

    if (arg1 == "-A" || arg1 == "--algorithm")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        algorithm = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-a" || arg1 == "--accent")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        accents.push_back (argv[i + 1]);
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-b" || arg1 == "--broadcast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::BROADCAST;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-d" || arg1 == "--domain")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        settings.write_domain = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-e" || arg1 == "--rebroadcasts")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        int hops;
        std::stringstream buffer (argv[i + 1]);
        buffer >> hops;

        settings.set_rebroadcast_ttl (hops);
        settings.enable_participant_ttl (hops);
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-f" || arg1 == "--logfile")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        madara::logger::global_logger->add_file (argv[i + 1]);
        gams::loggers::global_logger->add_file (argv[i + 1]);
      }
      else
        print_usage (argv[0]);

      ++i;
    }

    else if (arg1 == "-i" || arg1 == "--id")
    {
      if (i + 1 < argc && argv[i +1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> settings.id;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "--madara-level")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        int level;
        buffer >> level;
        madara::logger::global_logger->set_level (level);
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "--gams-level")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        int level;
        buffer >> level;
        gams::loggers::global_logger->set_level (level);
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-L" || arg1 == "--loop-time")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> loop_time;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-m" || arg1 == "--multicast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::MULTICAST;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-M" || arg1 == "--madara-file")
    {
      bool files = false;
      ++i;
      for (;i < argc && argv[i][0] != '-'; ++i)
      {
        std::string filename = argv[i];
        if (madara::utility::file_exists (filename))
        {
          madara_commands += madara::utility::file_to_string (filename);
          madara_commands += ";\n";
          files = true;
        }
      }
      --i;

      if (!files)
        print_usage (argv[0]);
    }
    else if (arg1 == "-n" || arg1 == "--num_agents")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> num_agents;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-o" || arg1 == "--host")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        host = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-p" || arg1 == "--platform")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        platform = argv[i + 1];
        plat_set = true;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-P" || arg1 == "--period")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> period;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-q" || arg1 == "--queue-length")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> settings.queue_length;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-r" || arg1 == "--reduced")
    {
      settings.send_reduced_message_header = true;
    }
    else if (arg1 == "-t" || arg1 == "--target")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        file_path = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-u" || arg1 == "--udp")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::UDP;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else
    {
      print_usage (argv[0]);
    }
  }
}

// perform main logic of program
int main (int argc, char ** argv)
{
  // handle all user arguments
  handle_arguments (argc, argv);
  
  // create knowledge base and a control loop
  madara::knowledge::KnowledgeBase knowledge;
  
  // if you only want to use custom transports, delete following
  knowledge.attach_transport (host, settings);

  // create containers
  Containers containers(knowledge);

  // open serial port to boat arduino
  std::shared_ptr<asio::serial_port> port;
  asio::io_service io;
  port = std::make_shared<asio::serial_port>(io);
  std::string port_name = PORT_NAME;
  asio::error_code ec;
  port->open(port_name, ec);
  if (!ec) 
  {
    if (port->is_open()) 
    {
        printf("port is open\n");
        port->set_option(asio::serial_port_base::baud_rate(BAUD_RATE));
    }
    else 
    {
      printf("ERROR: port->is_open() returned false\n");
      return -1;
    }
  }
  else 
  {
    printf("ERROR: port->open() failed:  %s\n", ec.message().c_str());
    return -1;            
  }
  
  // begin transport creation 
  // end transport creation
  
  controllers::BaseController controller (knowledge);
  madara::threads::Threader threader (knowledge);

  // initialize variables and function stubs
  controller.init_vars (settings.id, num_agents);
  
  std::vector <std::string> aliases;
  
  // begin adding custom algorithm factories
  // end adding custom algorithm factories

  // begin adding custom platform factories
  // add boat factory
  aliases.clear ();
  aliases.push_back ("boat");
  controller.add_platform_factory (aliases,
    new platforms::boatFactory ());

  // end adding custom platform factories
  
  // read madara initialization
  if (madara_commands != "")
  {
    knowledge.evaluate (madara_commands,
      madara::knowledge::EvalSettings(false, true));
  }
  
  // initialize the platform and algorithm
  // default to platform in knowledge base if platform not set in command line
  if (!plat_set && knowledge.exists (KNOWLEDGE_BASE_PLATFORM_KEY))
    platform = knowledge.get (KNOWLEDGE_BASE_PLATFORM_KEY).to_string ();
  controller.init_platform (platform);
  controller.init_algorithm (algorithm);

  // add any accents
  for (unsigned int i = 0; i < accents.size (); ++i)
  {
    controller.init_accent (accents[i]);
  }

  // begin thread creation
  threader.run (1, "localization", new threads::localization());
  threader.run (1, "control", new threads::control());
  threader.run (1, "analytics", new threads::analytics());
  threader.run (1, "sensing", new threads::sensing()); // thread for sensors that go directly to the odroid, not through the arduino
  threader.run (20.0, "JSON_read", new threads::JSON_read(port, containers)); // messages that come from the arduino
  threader.run (20.0, "JSON_write", new threads::JSON_write()); // messages that go to the arduino
  threader.run (1, "kb_print", new threads::kb_print());
  // end thread creation
  
  // run a mape loop for algorithm and platform control
  controller.run (period, loop_time);

  // terminate all threads after the controller
  threader.terminate ();
  
  // wait for all threads
  threader.wait ();
  
  // print all knowledge values
  knowledge.print ();

  return 0;
}

