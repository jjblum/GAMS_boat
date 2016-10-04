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
#include "threads/analytics.h"
#include "threads/compass_spoofer.h"
#include "threads/control.h"
#include "threads/gps_spoofer.h"
#include "threads/dynamite.h"
//#include "threads/ahrs.h"
//#include "threads/gps.h"
#include "threads/kb_print.h"
#include "threads/localization.h"
#include "threads/random_motor_signals.h"
#include "threads/sensing.h"
#include "threads/operator_watchdog.h"
#include "threads/ode_simulation_spoofer.h"
// end thread includes

// begin transport includes
// end transport includes

// begin other includes
#include "boat_containers.h"
#include <memory>
#include <chrono>
#include <thread>
#include "design.h"
#include "lutra_tank.h"
#include "lutra_articulated_fan.h"

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
std::string algorithm ("null");
std::vector <std::string> accents;

// controller variables
double period (1.0);
double loop_time (50.0);

#define GAMS_RUN_HZ 10.0
#define GAMS_SEND_HZ 5.0

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
  Containers containers(knowledge, settings.id);

  // begin transport creation 
  // end transport creation
  
  controllers::BaseController controller (knowledge);
  madara::threads::Threader threader (knowledge);

  // initialize variables and function stubs
  //controller.init_vars (settings.id, num_agents);
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

  // check if it is a boat platform via a dynamic_cast not returning null, and if it is a valid pointer, set the containers
  gams::platforms::BasePlatform * platform_ptr = controller.get_platform();  
  platforms::boat * boat_platform_ptr;
  boat_platform_ptr = dynamic_cast< platforms::boat * > (platform_ptr);
  if (boat_platform_ptr != nullptr)
  {
    boat_platform_ptr->set_containers(containers);
  }
  
  // add any accents
  for (unsigned int i = 0; i < accents.size (); ++i)
  {
    controller.init_accent (accents[i]);
  }
  
  // add any logging
  madara::logger::global_logger->add_file ("madara_log.txt");
  gams::loggers::global_logger->add_file ("gams_log.txt");
  //madara_logger_ptr_log(gams::loggers::global_logger.get(), 0, "HEYYYYYYY\n");
  //gams::loggers::global_logger->log(0, "THERE\n");
  //printf("GAMS LOG LEVEL: %d\n", (int)gams::loggers::global_logger->get_level());
  
  // set up boat Design object
  std::shared_ptr<designs::Design> design;
  int design_type = (int)containers.design_type.to_integer();
  switch (design_type)
  {
    case (int)DESIGN_TYPE::LUTRA_TANK:
      printf("Using a LUTRA_TANK design\n");
      design = std::dynamic_pointer_cast<designs::Design>(std::make_shared<designs::LutraTank>());
      break;
    case (int)DESIGN_TYPE::LUTRA_ARTICULATED_FAN:
      printf("Using a LUTRA_ARTICULATED_FAN design\n");
      design = std::dynamic_pointer_cast<designs::Design>(std::make_shared<designs::LutraArticulatedFan>());
      break;
    default:
      printf("WARNING: unknown boat design type. Defaults to using a LUTRA_TANK design\n");
      design = std::dynamic_pointer_cast<designs::Design>(std::make_shared<designs::LutraTank>());
      break;
  }
  if (design == nullptr)
  {
    printf("Design object dynamic cast returned a null pointer\n");
    return -1;
  }

  //Set arming sinal
    
  threads::localization * localization_thread = new threads::localization(containers); // separated out b/c i want to try callbacks and the caller needs a reference to the callee's instance

  // begin thread creation
  //threader.run (1, "analytics", new threads::analytics ());
  //threader.run (10.0, "compass_spoofer", new threads::compass_spoofer (localization_thread));
  threader.run (20.0, "control", new threads::control (containers, design));
  //threader.run (5.0, "gps_spoofer", new threads::gps_spoofer (containers, localization_thread));
  //threader.run (0.0, "dynamite", new threads::dynamite (port, containers, localization_thread));
  //threader.run (0.0, "gps", new threads::gps (port, containers));
  //threader.run (0.0, "ahrs", new threads::ahrs (port, containers));
  //threader.run (1.0, "kb_print", new threads::kb_print ());
  threader.run (50.0, "localization", localization_thread);
  //threader.run (1.0, "random_motor_signals", new threads::random_motor_signals (containers));
  //threader.run (10.0, "ahrs", new threads::AHRS (AHRS, localization_thread));
  //threader.run (1, "sensing", new threads::sensing ());
  //threader.run (1.0, "operator_watchdog", new threads::operator_watchdog(containers));
  //threader.run(100.0, "ODE_sim_spoofer", new threads::ODE_sim_spoofer (containers, design, localization_thread));
  // end thread creation
  
  // run a mape loop for algorithm and platform control
  controller.run_hz (GAMS_RUN_HZ, -1, GAMS_SEND_HZ);

  // terminate all threads after the controller
  threader.terminate ();
  
  // wait for all threads
  threader.wait ();
  
  // print all knowledge values
  knowledge.print ();

  return 0;
}

