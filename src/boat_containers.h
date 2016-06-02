#ifndef CONTAINERS_H
#define CONTAINERS_H

#include <string>

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "madara/knowledge/containers/Double.h"
#include "madara/knowledge/containers/Integer.h"
#include "madara/knowledge/containers/String.h"

namespace containers = madara::knowledge::containers;

class Containers {
public:
    Containers();
    Containers(madara::knowledge::KnowledgeBase &kb_, int id);
    ~Containers();
    
    // Localization and control stuff
    containers::NativeDoubleVector motor_signals;
    containers::NativeDoubleVector eastingNorthingHeading;
    containers::NativeDoubleVector location; // lat, long, elevation
    containers::NativeDoubleVector local_state; // local x, local y, theta, xdot, ydot, thetadot
    containers::Double thrustFraction;
    containers::Double headingFraction;
    containers::Double sufficientProximity;
    containers::Integer gpsZone;
    containers::Integer northernHemisphere;
    
    // Integer status stuff
    containers::Integer gps_init;
    containers::Integer compass_init;
    containers::Integer localized;
    containers::Integer heartbeat_gps;
    containers::Integer heartbeat_connectivity;
    containers::Integer heartbeat_operator;
    containers::Integer teleop_status;
    
    // misc. stuff
    containers::Double battery_voltage;
private:
    madara::knowledge::KnowledgeBase kb;
};





#endif  // CONTAINERS_H
