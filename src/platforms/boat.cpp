
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "boat.h"

// factory class for creating a boat 
gams::platforms::BasePlatform *
platforms::boatFactory::create (
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        gams::variables::Sensors * sensors,
        gams::variables::Platforms * platforms,
        gams::variables::Self * self)
{
  return new boat (knowledge, sensors, self);
}
        
// Constructor
platforms::boat::boat (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self)
: gams::platforms::BasePlatform (knowledge, sensors, self)
{
  // as an example of what to do here, create a coverage sensor
  if (knowledge && sensors)
  {
    // create a coverage sensor
    gams::variables::Sensors::iterator it = sensors->find ("coverage");
    if (it == sensors->end ()) // create coverage sensor
    {
      // get origin
      gams::utility::GPSPosition origin;
      madara::knowledge::containers::NativeDoubleArray origin_container;
      origin_container.set_name ("sensor.coverage.origin", *knowledge, 3);
      origin.from_container (origin_container);

      // establish sensor
      gams::variables::Sensor* coverage_sensor =
        new gams::variables::Sensor ("coverage", knowledge, 2.5, origin);
      (*sensors)["coverage"] = coverage_sensor;
    }
    (*sensors_)["coverage"] = (*sensors)["coverage"];

    ///////////////////////////////////////////////////////////////////////////////
    //status_.init_vars(*knowledge, get_id());
    status_.init_vars(*knowledge, "");
    status_.movement_available = 1;
    ///////////////////////////////////////////////////////////////////////////////
  }
}


// Destructor
platforms::boat::~boat ()
{
}


// Polls the sensor environment for useful information. Required.
int platforms::boat::sense (void)
{
  //printf("platform.sense()\n");
  return gams::platforms::PLATFORM_OK;
}


// Analyzes platform information. Required.
int
platforms::boat::analyze (void)
{
  //printf("platform.analyze()\n");
  containers.heartbeat_connectivity = 1;
  return gams::platforms::PLATFORM_OK;
}


// Gets the name of the platform. Required.
std::string
platforms::boat::get_name () const
{
  return "boat";
}


// Gets the unique identifier of the platform.
std::string
platforms::boat::get_id () const
{
  return "boat";
}


// Gets the position accuracy in meters. Optional.
double
platforms::boat::get_accuracy (void) const
{
  // will depend on your localization capabilities for robotics
  return containers.sufficientProximity.to_double();
}

// Gets Location of platform, within its parent frame. Optional.
gams::utility::Location
platforms::boat::get_location () const
{
  gams::utility::Location result;
  
  return result;
}


// Gets Rotation of platform, within its parent frame. Optional.
gams::utility::Rotation
platforms::boat::get_rotation () const
{
  gams::utility::Rotation result(0., 0., containers.eastingNorthingHeading[2]);
  
  return result;
}


// Gets sensor radius. Optional.
double
platforms::boat::get_min_sensor_range () const
{
  // should be in square meters
  return 0.0;
}

// Gets move speed. Optional.
double
platforms::boat::get_move_speed () const
{
  // should be in meters/s
  return 0.0;
}

// Instructs the agent to return home. Optional.
int
platforms::boat::home (void)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/

  self_->agent.dest.set(0, self_->agent.dest[0]);
  self_->agent.dest.set(1, self_->agent.dest[1]);
  self_->agent.source.set(0, containers.eastingNorthingHeading[0]);
  self_->agent.source.set(1, containers.eastingNorthingHeading[1]);
  
  return gams::platforms::PLATFORM_IN_PROGRESS;
}


// Instructs the agent to land. Optional.
int
platforms::boat::land (void)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_IN_PROGRESS;
}


// Moves the platform to a location. Optional.
int
platforms::boat::move (
  const gams::utility::Location & location,
  double epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
   
   double lat = location.lat();
   double lng = location.lng();
   
   printf("platform.move():  lat = %f, lng = %f\n", lat, lng);

   GeographicLib::GeoCoords coord(lat, lng, containers.gpsZone.to_integer());
   double easting = coord.Easting();
   double northing = coord.Northing();   
   
   if (easting != self_->agent.dest[0] || northing != self_->agent.dest[1])
   {
     printf("platform.move():  NEW DESTINATION\n");
    printf("platform.move():  new x = %f,  new y = %f,  old x = %f,  old y = %f\n", 
//       easting, northing, self_->agent.dest[0], self_->agent.dest[1]);   

     // update source to prior destination
     self_->agent.source.set(0, self_->agent.dest[0]);
     self_->agent.source.set(1, self_->agent.dest[1]);
     
     // new destination
     self_->agent.dest.set(0, easting);
     self_->agent.dest.set(1, northing);    
   }
   
  return gams::platforms::PLATFORM_MOVING;
}


// Rotates the platform to match a given angle. Optional.
int
platforms::boat::rotate (
  const gams::utility::Rotation & target,
  double epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_MOVING;
}


// Moves the platform to a pose (location and rotation). Optional.
int
platforms::boat::pose (const gams::utility::Pose & target,
  double loc_epsilon, double rot_epsilon)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_MOVING;
}

// Pauses movement, keeps source and dest at current values. Optional.
void
platforms::boat::pause_move (void)
{
}


// Set move speed. Optional.
void
platforms::boat::set_move_speed (const double& speed)
{
}


// Stops movement, resetting source and dest to current location. Optional.
void
platforms::boat::stop_move (void)
{
}

// Instructs the agent to take off. Optional.
int
platforms::boat::takeoff (void)
{
  return gams::platforms::PLATFORM_OK;
}

void platforms::boat::set_containers(Containers & containers_)
{
  containers = containers_;
}
