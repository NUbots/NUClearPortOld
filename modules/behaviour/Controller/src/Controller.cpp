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

#include "Controller.h"

#include "messages/motion/ServoWaypoint.h"

namespace modules {
    namespace behaviour {
        
        using messages::input::ServoID;
        using ServoWaypoint = messages::motion::ServoWaypointX;
        using messages::behaviour::RegisterAction;
        using messages::behaviour::ActionPriorites;
        using messages::behaviour::ServoCommand;
        using messages::behaviour::LimbID;
        
        // So we don't need a huge long type declaration everywhere...
        using iterators = std::pair<std::vector<std::reference_wrapper<RequestItem>>::iterator, std::vector<std::reference_wrapper<RequestItem>>::iterator>;
        
        Controller::Controller(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            
            on<Trigger<RegisterAction>, Options<Sync<Controller>>>("Action Registration", [this] (const RegisterAction& action) {
                
                if(action.id == 0) {
                    throw std::runtime_error("Action ID 0 is reserved for internal use");
                }
                else if(requests.find(action.id) != std::end(requests)) {
                    throw std::runtime_error("The passed action ID has already been registered");
                };
                
                // Make our request object
                requests[action.id] = std::make_unique<Request>(action.id, action.start, action.kill, action.completed);
                auto& request = requests[action.id];
                
                // In order for our references to hold valid, we need to never reallocate this
                request->items.reserve(action.limbSet.size());
                
                // Make our request items
                for(const auto& set : action.limbSet) {
                    request->items.emplace_back(*request, request->items.size(), set.first, set.second);
                    
                    
                    // Put our request in the correct queue
                    for(auto& l : request->items.back().limbSet) {
                        actions[uint(l)].push_back(std::ref(request->items.back()));
                    }
                }
                
                // Find the main element
                auto maxRequest = std::max_element(std::begin(request->items), std::end(request->items),
                                                   [] (const RequestItem& a, const RequestItem& b) {
                                                       return a.priority < b.priority;
                                                   });
                
                // Set the main element to this one
                request->mainElement = maxRequest->index;
                request->maxPriority = maxRequest->priority;
            });
            
            on<Trigger<Startup>, Options<Sync<Controller>>>("Initial Action Selection", [this] (const Startup&) {
                
                // Pick our first action to take
                selectAction();
            });
            
            on<Trigger<ActionPriorites>, Options<Sync<Controller>>>("Action Priority Update", [this] (const ActionPriorites& update) {
                
                auto& request = requests[update.id];
                
                // Find the largest priority
                auto maxEl = std::max_element(std::begin(update.priorities), std::end(update.priorities));
                
                // Find its index
                uint mainElement = std::distance(std::begin(update.priorities), maxEl);
                
                // Unless we need to, try not to run the expensive subsumption algorithm
                bool reselect;
                
                // If our main changed we have to reselect
                reselect = mainElement != request->mainElement;
                
                request->mainElement = mainElement;
                request->maxPriority = *maxEl;
                
                
                // Perform our update
                for(uint i = 0; i < request->items.size(); ++i) {
                    
                    bool up = request->items[i].priority < update.priorities[i];
                    bool down = request->items[i].priority > update.priorities[i];
                    bool active = request->items[i].active;
                    
                    // Short circuit if we can
                    // TODO see if we can add more here
                    reselect |= (up != down) && ((active && down) || (!active && up));
                    
                    // Update our priority
                    request->items[i].priority = update.priorities[i];
                }
                
                if(reselect) {
                    // Select our new action
                    selectAction();
                }
                
            });
            
            // For single waypoints
            on<Trigger<ServoCommand>>([this](const ServoCommand& point) {
                
                // Make a vector of the command
                auto points = std::make_unique<std::vector<ServoCommand>>();
                points->push_back(point);
                emit<Scope::DIRECT>(std::move(points));
            });
            
            on<Trigger<std::vector<ServoCommand>>, Options<Sync<Controller>>>("Command Filter", [this] (const std::vector<ServoCommand>& commands) {
                
                for (auto& command : commands) {
                    
                    // Check if we have access
                    if (this->limbAccess[uint(messages::behaviour::limbForServo(command.id))] == command.source) {
                        
                        // Get our queue
                        auto& queue = commandQueues[uint(command.id)];
                        
                        // Clear commands until we get back one that we are after
                        while(!queue.empty() && queue.back().time > command.time) {
                            queue.pop_back();
                        }
                        
                        // Push our command onto the queue
                        queue.push_back(command);
                    }
                }
            });
            
            on<Trigger<Every<60, Per<std::chrono::seconds>>>, Options<Sync<Controller>>>([this] (const time_t& now) {
                
                std::list<ServoID> emptiedQueues;
                std::unique_ptr<std::vector<ServoWaypoint>> waypoints;
                
                for(auto& queue : commandQueues) {
                    
                    if (!queue.empty() && queue.front().time < now) {
                        
                        // Store our ID (if we need it)
                        auto id = queue.front().id;
                        
                        queue.pop_front();
                        
                        if(queue.empty()) {
                            // Keep track of what we have emptied
                            emptiedQueues.push_back(id);
                        }
                    }
                    
                    // Dirty hack, we set source to 0 when it's processed
                    if(!queue.empty() && queue.front().source != 0) {
                        
                        auto& command = queue.front();
                        
                        // Lazy initialize
                        if(!waypoints) {
                            waypoints = std::make_unique<std::vector<ServoWaypoint>>();
                        }
                        
                        // Add to our waypoints
                        waypoints->push_back({ command.time, command.id, command.position, command.gain });
                        
                        // Dirty hack the waypoint
                        command.source = 0;
                        
                    }
                }
                
                // Emit our waypoints
                if(waypoints) {
                    emit(std::move(waypoints));
                }
                
                if(!emptiedQueues.empty()) {
                    
                    std::map<size_t, std::set<ServoID>> completeMap;
                    
                    for (auto& servo : emptiedQueues) {
                        
                        // Get the id of the limb this was leased to
                        auto id = limbAccess[uint(messages::behaviour::limbForServo(servo))];
                        completeMap[id].insert(servo);
                    }
                    
                    for(const auto& e : completeMap) {
                        requests[e.first]->completed(e.second);
                    }
                }
            });
        }
        
        bool hasLimbs(const std::set<LimbID>& limbRequest, const std::map<LimbID, iterators>& limbAvailable) {
            
            // Get the available limbs and the requested limbs
            auto available = std::begin(limbAvailable); // first
            auto request   = std::begin(limbRequest); // second
            
            // Check that every limb in request is available
            for (; request != std::end(limbRequest); ++available)
            {
                // If we reach the end of the request, or we don't have the limb return false
                if (available == std::end(limbAvailable) || *request < available->first) {
                    return false;
                }
                // If our element is after our requested element check the next limb
                if (available->first >= *request) {
                    ++request;
                }
            }
            return true;
        }
        
        void Controller::selectAction() {
            
            // Sort each of the lists to choose a new item
            for(auto& l : actions) {
                std::stable_sort(std::begin(l), std::end(l), [] (const RequestItem& a, const RequestItem& b) {
                    return a.priority > b.priority;
                });
            }
            
            // Set the active flags on the current actions to false
            for (auto& action : currentActions) {
                action.get().active = false;
                action.get().group.active = false;
            }
            
            // Get iterators to each of the actions on the limbs
            std::map<LimbID, iterators> limbs =
            {
                std::make_pair(LimbID::LEFT_LEG    , std::make_pair(std::begin(actions[uint(LimbID::LEFT_LEG)])  , std::end(actions[uint(LimbID::LEFT_LEG)])))
                , std::make_pair(LimbID::RIGHT_LEG , std::make_pair(std::begin(actions[uint(LimbID::RIGHT_LEG)]) , std::end(actions[uint(LimbID::RIGHT_LEG)])))
                , std::make_pair(LimbID::LEFT_ARM  , std::make_pair(std::begin(actions[uint(LimbID::LEFT_ARM)])  , std::end(actions[uint(LimbID::LEFT_ARM)])))
                , std::make_pair(LimbID::RIGHT_ARM , std::make_pair(std::begin(actions[uint(LimbID::RIGHT_ARM)]) , std::end(actions[uint(LimbID::RIGHT_ARM)])))
                , std::make_pair(LimbID::HEAD      , std::make_pair(std::begin(actions[uint(LimbID::HEAD)])      , std::end(actions[uint(LimbID::HEAD)])))
            };
            
            // Our new actions
            std::vector<std::reference_wrapper<RequestItem>> newActions;
            
            // We keep adding actions while we have limbs to add
            while (!limbs.empty()) {
                
                // Find the largest iterator task from all the limbs
                auto maxIt = std::max_element(std::begin(limbs), std::end(limbs),
                                              [](const std::pair<LimbID, iterators>& a,
                                                 const std::pair<LimbID, iterators>& b) {
                                                  
                                                  // Empty lists go to the end
                                                  if(a.second.first == a.second.second) {
                                                      return true;
                                                  }
                                                  else if(b.second.first == b.second.second) {
                                                      return false;
                                                  }
                                                  // The smaller priority loses
                                                  return a.second.first->get().priority < b.second.first->get().priority;
                                              });
                
                // If we ran out of possible actions for this limb remove it
                if(maxIt->second.first == maxIt->second.second) {
                    limbs.erase(maxIt);
                }
                
                // Otherwise this is a candidate for selection
                else {
                    
                    // This our action item we are looking at
                    auto& action = maxIt->second.first->get();
                    
                    // Are we already active (from previous main activation)
                    // Are we the main action?
                    if(((action.index == action.group.mainElement) || action.group.active)
                       // Do we have the needed limbs
                       && hasLimbs(action.limbSet, limbs)) {
                        
                        // Activate this group and item
                        action.active = true;
                        action.group.active = true;
                        
                        // Push this action onto our list of actions
                        newActions.push_back(std::ref(action));
                        
                        // Remove the limbs that we have just allocated
                        for(auto& limb : action.limbSet) {
                            limbs.erase(limbs.find(limb));
                        }
                        
                    }
                    // This request isn't suitable, move to the next one
                    else {
                        ++(maxIt->second.first);
                    }
                }
            }
            
            // Reset the limb access
            for (auto& l : limbAccess) {
                l = 0;
            }
            
            // Set the permissions for a limb according to our allocations
            for (auto& command : newActions) {
                for(auto& l : command.get().limbSet) {
                    limbAccess[uint(l)] = command.get().group.id;
                }
            }
            
            // This comparator will sort our list so we can compare it with the current actions
            std::function<bool (const RequestItem&, const RequestItem&)> comp =
            [] (const RequestItem& a, const RequestItem& b) {
                if(a.group.id < b.group.id) {
                    return true;
                }
                else if(a.group.id == b.group.id) {
                    return a.index < b.index;
                }
                else {
                    return false;
                }
            };
            
            // Sort our list
            std::sort(std::begin(newActions), std::end(newActions), comp);
            
            // These are used to work out the difference
            std::vector<std::reference_wrapper<RequestItem>> start;
            std::map<size_t, std::set<LimbID>> startMap;
            
            std::vector<std::reference_wrapper<RequestItem>> kill;
            std::map<size_t, std::set<LimbID>> killMap;
            
            // We will never have more then 5
            start.reserve(5);
            kill.reserve(5);

            // Find newly added actions
            std::set_difference(std::begin(newActions), std::end(newActions),
                                std::begin(currentActions), std::end(currentActions),
                                std::back_inserter(start), comp);
            
            // Find now deleted actions
            std::set_difference(std::begin(currentActions), std::end(currentActions),
                                std::begin(newActions), std::end(newActions),
                                std::back_inserter(kill), comp);
            
            // Fill up our map with a list of limbs to kill (and the controllers for it)
            for (const auto& k : kill) {
                for (const auto& l : k.get().limbSet) {
                    killMap[k.get().group.id].insert(l);
                }
            }
            
            // Fill up our map with a list of limbs to start (and the controllers for it)
            for (const auto& s : start) {
                for (const auto& l : s.get().limbSet) {
                    startMap[s.get().group.id].insert(l);
                }
            }
            
            // Execute all our kill commands
            for (const auto& k : killMap) {
                requests[k.first]->kill(k.second);
                
                // Clear our queues for this limb
                for(const auto& limb : k.second) {
                    for (const auto& servo : messages::behaviour::servosForLimb(limb)) {
                        commandQueues[uint(servo)].clear();
                    }
                }
            }
            
            // Execute our start commands
            for (const auto& s : startMap) {
                requests[s.first]->start(s.second);
            }
            
            // Our actions are now these new actions
            currentActions = std::move(newActions);
        }
        
    }  // behaviours
}  // modules
