/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_PLANNING_KICKPLANNER_H
#define MODULES_BEHAVIOUR_PLANNING_KICKPLANNER_H

#include <armadillo>

#include <nuclear>
namespace modules {
namespace behaviour {
namespace planning {

    class KickPlanner : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and setup the KickPlanner reactor.
        explicit KickPlanner(std::unique_ptr<NUClear::Environment> environment);
        static constexpr const char* CONFIGURATION_PATH = "KickPlanner.json";

    private:
       	arma::vec2 TARGET_FIELD_POS;
       	float MIN_BALL_DISTANCE;
       	float KICK_CORRIDOR_WIDTH;
       	float KICK_FORWARD_ANGLE_LIMIT;
       	float KICK_SIDE_ANGLE_LIMIT;


    };

}
}
}


#endif