#ifndef MODULES_DARWINSENSORS_H
#define MODULES_DARWINSENSORS_H

#include <NUClear.h>

namespace modules {

    class DarwinSensors : public NUClear::Reactor {
    public:
        DarwinSensors(NUClear::PowerPlant& plant);
    };
}
#endif
