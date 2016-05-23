#include "boat_containers.h"

Containers::Containers(madara::knowledge::KnowledgeBase &kb_) 
: kb(kb_)
{ 
    battery_voltage.set_name(".battery_voltage", kb);
    battery_voltage = 0.0;
    
    motor_signals.set_name(".motor_signals", kb);
    std::vector<double> motor_signal_start = {0.0, 0.0};
    motor_signals.set(motor_signal_start);

    gps_init.set_name("gps_initialized", kb);
    gps_init = 0;

    compass_init.set_name("compass_initialized", kb);
    compass_init = 0;

    localized.set_name("localized", kb);
    localized = 0;
}

Containers::~Containers() {  }
