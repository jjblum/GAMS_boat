#include "boat_containers.h"

Containers::Containers(madara::knowledge::KnowledgeBase &kb_) 
: kb(kb_)
{ 
    battery_voltage.set_name(".battery_voltage", kb);
    battery_voltage = 0.0;
    
    motor_signals.set_name(".motor_signals", kb);
    std::vector<double> motor_signal_start = {0.0, 0.0};
    motor_signals.set(motor_signal_start);
}

Containers::~Containers() {  }
