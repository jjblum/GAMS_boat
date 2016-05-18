#ifndef CONTAINERS_H
#define CONTAINERS_H

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "madara/knowledge/containers/Double.h"

namespace containers = madara::knowledge::containers;

class Containers {
public:
    Containers(madara::knowledge::KnowledgeBase &kb_);
    ~Containers();
    containers::Double battery_voltage;
    containers::NativeDoubleVector motor_signals;
private:
    madara::knowledge::KnowledgeBase kb;
};





#endif  // CONTAINERS_H
