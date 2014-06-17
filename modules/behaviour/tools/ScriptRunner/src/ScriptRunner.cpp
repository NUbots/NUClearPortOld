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

#include "ScriptRunner.h"

#include "messages/motion/Script.h"
#include "messages/behaviour/Action.h"

namespace modules {
    namespace behaviour {
        namespace tools {

            using messages::motion::ExecuteScriptByName;
            using messages::behaviour::RegisterAction;
            using messages::behaviour::LimbID;
            using messages::input::ServoID;

            struct ExecuteNextScript {};

            void ScriptRunner::executeNextScript() {

                // If we have a script to execute
                if(!scripts.empty()) {

                    // Get it and emit it
                    auto script = scripts.front();
                    emit(std::make_unique<ExecuteScriptByName>(id, script));
                    scripts.pop();
                }
                // Otherwise we are done, shutdown
                else {
                    powerplant.shutdown();
                }
            }

            ScriptRunner::ScriptRunner(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

                // Get the scripts to run from the command line
                on<Trigger<CommandLineArguments>>([this](const std::vector<std::string>& args) {
                    NUClear::log<NUClear::INFO>("Executing: ", args.size(), " scripts");

                    for(size_t i = 1; i < args.size(); ++i) {
                        NUClear::log<NUClear::INFO>("Queueing script ", args[i]);
                        scripts.push(args[i]);
                    }
                });

                on<Trigger<ExecuteNextScript>>([this](const ExecuteNextScript&) {
                    executeNextScript();
                });

                emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
                    id,
                    "script_runner",
                    { std::pair<float, std::set<LimbID>>(1, { LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD }) },
                    [this] (const std::set<LimbID>&) {
                        emit(std::make_unique<ExecuteNextScript>());
                    },
                    [this] (const std::set<LimbID>&) {
                        // We should always be the only running thing
                    },
                    [this] (const std::set<ServoID>&) {
                        emit(std::make_unique<ExecuteNextScript>());
                    }
                }));
            }

        }  // tools
    }  // behaviours
}  // modules
