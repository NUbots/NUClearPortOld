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

#ifndef MESSAGES_SCRIPT_H
#define MESSAGES_SCRIPT_H

#include <chrono>
#include <yaml-cpp/yaml.h>

#include "messages/input/ServoID.h"

namespace messages {
    namespace motion {

        /**
         * TODO document
         *
         * @author Trent Houliston
         * @author Trent Houliston
         */
        struct Script {
            struct Frame {
                struct Target {
                    input::ServoID id;
                    float position;
                    float gain;
                };
                NUClear::clock::duration duration;
                std::vector<Target> targets;
            };
            std::vector<Frame> frames;
        };

        Script operator +(const Script& s1, const Script& s2){
            Script s;
            s.frames.insert(s.frames.end(), s1.frames.begin(), s1.frames.end());
            s.frames.insert(s.frames.end(), s2.frames.begin(), s2.frames.end());
            return s;
        }

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct ExecuteScriptByName {
            ExecuteScriptByName(const size_t& id, const std::string script, NUClear::clock::time_point start = NUClear::clock::now()) : sourceId(id), scripts({script}), start(start) {};
            ExecuteScriptByName(const size_t& id, const std::vector<std::string> scripts, NUClear::clock::time_point start = NUClear::clock::now()) : sourceId(id), scripts(scripts), start(start) {};
            size_t sourceId;
            std::vector<std::string> scripts;
            NUClear::clock::time_point start;
        };

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct ExecuteScript {
            ExecuteScript(const size_t& id, const Script& script, NUClear::clock::time_point start = NUClear::clock::now()) : sourceId(id), scripts({script}), start(start) {};
            ExecuteScript(const size_t& id, const std::vector<Script>& scripts, NUClear::clock::time_point start = NUClear::clock::now()) : sourceId(id), scripts(scripts), start(start) {};
            size_t sourceId;
            std::vector<Script> scripts;
            NUClear::clock::time_point start;
        };

    }  // motion
}  // messages

namespace YAML {
    template<>
    struct convert<messages::motion::Script::Frame::Target> {
        static Node encode(const messages::motion::Script::Frame::Target& rhs) {
            Node node;

            node["id"] = messages::input::stringFromId(rhs.id);
            node["position"] = rhs.position;
            node["gain"] = rhs.gain;

            return node;
        }

        static bool decode(const Node& node, messages::motion::Script::Frame::Target& rhs) {

            rhs = { messages::input::idFromString(node["id"]), node["position"].as<float>(), node["gain"].as<float>() };
            return true;
        }
    };

    template<>
    struct convert<messages::motion::Script::Frame> {
        static Node encode(const messages::motion::Script::Frame& rhs) {
            Node node;

            node["duration"] = std::chrono::duration_cast<std::chrono::milliseconds>(rhs.duration).count();
            node["targets"] = rhs.targets;

            return node;
        }

        static bool decode(const Node& node, messages::motion::Script::Frame& rhs) {

            int millis = node["duration"].as<int>();
            std::chrono::milliseconds duration(millis);

            std::vector<messages::motion::Script::Frame::Target> targets = node["targets"].as<std::vector<messages::motion::Script::Frame::Target>>();

            rhs = { duration, std::move(targets) };
            return true;
        }
    };

    template<>
    struct convert<messages::motion::Script> {
        static Node encode(const messages::motion::Script& rhs) {
            Node node;

            node = rhs.frames;

            return node;
        }

        static bool decode(const Node& node, messages::motion::Script& rhs) {
            std::vector<messages::motion::Script::Frame> frames = node.as<std::vector<messages::motion::Script::Frame>>();
            rhs = { std::move(frames) };
            return true;
        }
    };
}

#endif
