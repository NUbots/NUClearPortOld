/*
 * This file is part of NCursesReactor.
 *
 * NCursesReactor is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * NCursesReactor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with NCursesReactor.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_SCRIPTTUNER_H
#define MODULES_SCRIPTTUNER_H

#include <NUClear.h>

namespace modules {

    class ScriptTuner : public NUClear::Reactor {

    private:
        enum Selection {
            SCRIPTNAME       = 2,
            FRAME            = 3,
            DURATION         = 4,
            HEAD_PAN         = 6,
            HEAD_TILT        = 7,
            R_SHOULDER_PITCH = 8,
            L_SHOULDER_PITCH = 9,
            R_SHOULDER_ROLL  = 10,
            L_SHOULDER_ROLL  = 11,
            R_ELBOW          = 12,
            L_ELBOW          = 13,
            R_HIP_YAW        = 14,
            L_HIP_YAW        = 15,
            R_HIP_ROLL       = 16,
            L_HIP_ROLL       = 17,
            R_HIP_PITCH      = 18,
            L_HIP_PITCH      = 19,
            R_KNEE           = 20,
            L_KNEE           = 21,
            R_ANKLE_PITCH    = 22,
            L_ANKLE_PITCH    = 23,
            R_ANKLE_ROLL     = 24,
            L_ANKLE_ROLL     = 25
        };


        volatile bool running;
        void run();
        void kill();
    public:
        explicit ScriptTuner(NUClear::PowerPlant* plant);
    };
}
#endif

