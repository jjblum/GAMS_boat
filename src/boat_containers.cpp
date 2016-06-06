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
    motor_signals.set_name("motorCommands", kb);
    std::vector<double> motor_signal_start = {0.0, 0.0};
    motor_signals.set(motor_signal_start);
    
    eastingNorthingHeading.set_name(prefix + "eastingNorthingBearing", kb);
    std::vector<double> eastingNorthingHeading_start = {0.0, 0.0, 0.0};
    eastingNorthingHeading.set(eastingNorthingHeading_start);
    
    location.set_name(prefix + "location", kb);
    std::vector<double> location_start = {0.0, 0.0, 0.0};
    location.set(location_start);
    
    local_state.set_name(prefix + "localState", kb);
    std::vector<double> local_state_start = {0., 0., 0., 0., 0., 0.};
    local_state.set(local_state_start);
    
    thrustFraction.set_name(prefix + "thrustFraction", kb);
    thrustFraction = 0.0;
    
    headingFraction.set_name(prefix + "bearingFraction", kb);
    headingFraction = 0.0;
    
    gpsZone.set_name(prefix + "gps_zone", kb);

    northernHemisphere.set_name(prefix + "northern_hemisphere", kb);
    
    design_type.set_name(prefix + "design_type", kb);    

    // Integer status stuff
    gps_init.set_name(prefix + "gpsInitialized", kb);
    gps_init = 0;

    compass_init.set_name(prefix + "compassInitialized", kb);
    compass_init = 0;

    localized.set_name(prefix + "localized", kb);
    localized = 0;
    
    heartbeat_gps.set_name(prefix + "gpsWatchdog", kb);
    heartbeat_gps = 1;
    
    heartbeat_connectivity.set_name(prefix + "connectivityWatchdog", kb);
    heartbeat_connectivity = 1;
    
    heartbeat_operator.set_name(prefix + "operatorHeartbeat", kb);
    heartbeat_operator = 0;
    
    teleop_status.set_name(prefix + "teleopStatus", kb);
    teleop_status = 1; // start in teleop mode
    
    // misc stuff
    battery_voltage.set_name(prefix + "batteryVoltage", kb);
    battery_voltage = 0.0;
    
    
}

Containers::~Containers() {  }
