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

#ifndef MODULES_BEHAVIOUR_PLANNERS_LOOKATBALL_H
#define MODULES_BEHAVIOUR_PLANNERS_LOOKATBALL_H

#include <nuclear>



namespace modules {
    namespace behaviour {
        namespace planners {

            /**
             * Executes a getup script if the robot falls over.
             *
             * @author Josiah Walker
             */
            class LookAtBall : public NUClear::Reactor {
            private:
                //const size_t id;

            public:
                explicit LookAtBall(std::unique_ptr<NUClear::Environment> environment);
                //static constexpr const char* CONFIGURATION_PATH = "Stand.json";
            };

        }  // planners
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_PLANNERS_LOOKATBALL_H

