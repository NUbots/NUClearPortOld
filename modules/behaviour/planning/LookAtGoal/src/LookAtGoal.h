/*
 * This file is part of the NUbots Codebase.
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

#ifndef MODULES_BEHAVIOUR_PLANNERS_LOOKATGOAL_H
#define MODULES_BEHAVIOUR_PLANNERS_LOOKATGOAL_H

#include <nuclear>
#include <armadillo>

namespace modules {
    namespace behaviour {
        namespace planning {

            /**
             * Executes a getup script if the robot falls over.
             *
             * @author Josiah Walker
             */
            class LookAtGoal : public NUClear::Reactor {
            private:
                //const size_t id;
                NUClear::clock::time_point timeSinceLastSeen;

                float GOAL_SEARCH_TIMEOUT_MILLISECONDS;
                arma::vec2 SCAN_YAW;
                arma::vec2 SCAN_PITCH;
                
                ReactionHandle lookAtReactionHandler;
            public:
                explicit LookAtGoal(std::unique_ptr<NUClear::Environment> environment);

                //static constexpr const char* CONFIGURATION_PATH = "Stand.json";
            };

        }  // planning
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_PLANNERS_LOOKATGOAL_H

