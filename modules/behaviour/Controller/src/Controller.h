/*
 * This file is part of Behaviour Controller.
 *
 * Behaviour Controller is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Behaviour Controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Behaviour Controller.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_CONTROLLER_H
#define MODULES_BEHAVIOUR_CONTROLLER_H

#include <nuclear>

#include <array>
#include <vector>
#include <map>
#include <set>
#include <list>

#include "messages/behaviour/Action.h"

namespace modules {
    namespace behaviour {

        struct RequestItem;

        struct Request {
            using callback = std::function<void (std::set<messages::behaviour::LimbID>)>;
            
            Request(size_t id, callback start, callback kill)
            : id(id)
            , active(false)
            , maxPriority(std::numeric_limits<float>::min())
            , start(start)
            , kill(kill) {}
            
            /// The ID of this request that will be sent with any motion commands
            size_t id;
            
            /// If the main element of this request is active
            bool active;
            
            /// The maximum priority for any of the items
            float maxPriority;
            
            /// The index of the main item
            size_t mainElement;
            
            /// The items in this list
            std::vector<RequestItem> items;
            
            /// The callback to execute when a new limb is started
            callback start;
            callback kill;
            std::function<void (std::set<messages::input::ServoID>)> completed;
        };

        struct RequestItem {
            
            RequestItem(Request& group, size_t index, float priority, const std::set<messages::behaviour::LimbID>& limbSet)
            : group(group)
            , index(index)
            , priority(priority)
            , limbSet(limbSet) {}
            
            Request& group;
            
            size_t index;
            
            bool active;
            
            float priority;
            std::set<messages::behaviour::LimbID> limbSet;
        };

        /**
         * Controls which of the behaviours are able to access motors.
         * 
         * @author Trent Houliston
         */
        class Controller : public NUClear::Reactor {
        private:
            std::array<std::vector<std::reference_wrapper<RequestItem>>, 5> actions;
            std::array<size_t, 5> limbAccess;
            std::map<size_t, std::unique_ptr<Request>> requests;
            std::vector<std::reference_wrapper<RequestItem>> currentActions;
            
            
            std::array<std::list<messages::behaviour::ServoCommand>, 20> commandQueues;
            
            void selectAction();
        public:
            explicit Controller(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOUR_CONTROLLER_H

