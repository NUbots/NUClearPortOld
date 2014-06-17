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

#include "Getup.h"

#include <cmath>
#include "messages/input/ServoID.h"
#include "messages/motion/Script.h"
#include "messages/behaviour/Action.h"
#include "messages/support/Configuration.h"
#include "messages/input/Sensors.h"

namespace modules {
    namespace behaviour {
        namespace skills {

            //internal only callback messages to start and stop our action
            struct ExecuteGetup{};
            struct KillGetup{};

            using messages::support::Configuration;
            using messages::input::Sensors;
            using messages::input::ServoID;
            using messages::motion::ExecuteScriptByName;
            using messages::behaviour::RegisterAction;
            using messages::behaviour::ActionPriorites;
            using messages::behaviour::LimbID;

            Getup::Getup(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment))
                , id(size_t(this) * size_t(this) - size_t(this))
                , gettingUp(false) {

                //do a little configurating
                on<Trigger<Configuration<Getup>>>([this] (const Configuration<Getup>& file){

                    //encode fallen angle as a cosine so we can compare it directly to the z axis value
                    double fallenAngleConfig = file.config["FALLEN_ANGLE"].as<double>();
                    FALLEN_ANGLE = cos(fallenAngleConfig);

                    //load priorities for the getup
                    GETUP_PRIORITY = file.config["GETUP_PRIORITY"].as<float>();
                    EXECUTION_PRIORITY = file.config["EXECUTION_PRIORITY"].as<float>();
                });

                on<Trigger<Sensors>, Options<Single>>([this] (const Sensors& sensors) {

                    //check if the orientation is smaller than the cosine of our fallen angle
                    if (!gettingUp && fabs(sensors.orientation(2,2)) < FALLEN_ANGLE) {
                        updatePriority(GETUP_PRIORITY);
                    }
                });

                on<Trigger<ExecuteGetup>, With<Sensors>>([this] (const ExecuteGetup&, const Sensors& sensors) {

                    gettingUp = true;

                    // Check with side we're getting up from
                    if (sensors.orientation(0,2) < 0.0) {
                        emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"StandUpFront.yaml","Stand.yaml"})));
                    }
                    else {
                        emit(std::make_unique<ExecuteScriptByName>(id, std::vector<std::string>({"StandUpBack.yaml","Stand.yaml"})));
                    }
                    updatePriority(EXECUTION_PRIORITY);
                });

                on<Trigger<KillGetup>, Options<Sync<Getup>>>([this] (const KillGetup&) {
                    gettingUp = false;
                    updatePriority(0);
                });

                emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
                    id,
                    "getup",
                    { std::pair<float, std::set<LimbID>>(0, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD }) },
                    [this] (const std::set<LimbID>&) {
                        emit(std::make_unique<ExecuteGetup>());
                    },
                    [this] (const std::set<LimbID>&) {
                        emit(std::make_unique<KillGetup>());
                    },
                    [this] (const std::set<ServoID>& servoSet) {
                        //HACK 2014 Jake Fountain, Trent Houliston
                        //TODO track set limbs and wait for all to finish
                        if(servoSet.find(ServoID::L_ANKLE_PITCH) != servoSet.end() ||
                           servoSet.find(ServoID::R_ANKLE_PITCH) != servoSet.end())
                        {

                        emit(std::make_unique<KillGetup>());

                        }
                    }
                }));
            }

            void Getup::updatePriority(const float& priority) {
                emit(std::make_unique<ActionPriorites>(ActionPriorites { id, { priority }}));
            }



        }  // skills
    }  // behaviours
}  // modules
