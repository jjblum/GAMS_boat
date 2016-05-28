#include "boat_containers.h"

Containers::Containers()
{
}

Containers::Containers(madara::knowledge::KnowledgeBase &kb_, int id) 
: kb(kb_)
{ 
    char prefix_[30];
    snprintf(prefix_, 30, "agent.%d.", id);
    std::string prefix(prefix_);

    // Localization and control stuff
    motor_signals.set_name(".motor_signals", kb);
    std::vector<double> motor_signal_start = {0.0, 0.0};
    motor_signals.set(motor_signal_start);
    
    eastingNorthingHeading.set_name(prefix + "easting_northing_bearing", kb);
    std::vector<double> eastingNorthingHeading_start = {0.0, 0.0, 0.0};
    eastingNorthingHeading.set(eastingNorthingHeading_start);
    
    location.set_name(prefix + "location", kb);
    std::vector<double> location_start = {0.0, 0.0, 0.0};
    location.set(location_start);
    
    thrustFraction.set_name(prefix + "thrust_fraction", kb);
    thrustFraction = 0.0;
    
    headingFraction.set_name(prefix + "heading_fraction", kb);
    headingFraction = 0.0;
    
    gpsZone.set_name(prefix + "gps_zone", kb);
    northernHemisphere.set_name(prefix + "northern_hemisphere", kb);

    // Integer status stuff
    gps_init.set_name(prefix + "gps_initialized", kb);
    gps_init = 0;

    compass_init.set_name(prefix + "compass_initialized", kb);
    compass_init = 0;

    localized.set_name(prefix + "localized", kb);
    localized = 0;
    
    heartbeat_gps.set_name(prefix + "heartbeat_gps", kb);
    heartbeat_gps = 1;
    
    heartbeat_connectivity.set_name(prefix + "heartbeat_connectivity", kb);
    heartbeat_connectivity = 1;
    
    heartbeat_operator.set_name(prefix + "heartbeat_operator", kb);
    heartbeat_operator = 0;
    
    teleop_status.set_name(prefix + "teleop_status", kb);
    teleop_status = 1; // start in teleop mode
    
    // misc stuff
    battery_voltage.set_name(prefix + "battery_voltage", kb);
    battery_voltage = 0.0;
    
    
}

Containers::~Containers() {  }
