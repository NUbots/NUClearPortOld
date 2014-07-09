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

#include "FixedWalk.h"

#include "utility/math/matrix.h"
#include "messages/motion/GetupCommand.h"

namespace modules {
    namespace behaviour {
        namespace planning {

            using messages::motion::WalkCommand;
            using messages::motion::WalkStartCommand;
            using messages::motion::WalkStopCommand;
        	using messages::motion::WalkStopped;
			using messages::behaviour::FixedWalkCommand;
			using messages::behaviour::FixedWalkFinished;
            using messages::motion::ExecuteGetup;
            using messages::motion::KillGetup;
			using messages::input::Sensors;


            FixedWalk::FixedWalk(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), active(false){
                // on<Trigger<Configuration<FixedWalk>>>([this] (const Configuration<FixedWalk>& file){
                // });

                on<Trigger<ExecuteGetup>>("FixedWalk::Getup", [this](const ExecuteGetup&){
                    //record fall time
                    segmentElapsedTimeBeforeFall = NUClear::clock::now() - segmentStart;
                    fallen = true;
                });

                on<Trigger<KillGetup>>("FixedWalk::Getup Finished", [this](const KillGetup&){
                    //getup finished
                    segmentStart = NUClear::clock::now() - segmentElapsedTimeBeforeFall;
                    fallen = false;
                });

                on< Trigger< Every<30, Per<std::chrono::seconds>>> , Options<Sync<FixedWalk>>, With<Sensors>>("Fixed Walk Manager", [this]( const time_t& t, const Sensors& sensors){
                    if(t > segmentStart + walkSegments.front().duration && active && !fallen){
                        //Move to next segment
                        segmentStart += walkSegments.front().duration;
                        walkSegments.pop();
                        if(walkSegments.empty()){
                            emit(std::make_unique<WalkCommand>());
                            emit(std::make_unique<WalkStopCommand>());
                            active = false;
                            return;
                        }
	        			beginningOrientation = sensors.orientation;
                    }
                    //Emit command
                    if(!walkSegments.empty()){
                		emit(getWalkCommand(walkSegments.front(), t-segmentStart, sensors));
                	}
                });

                on<Trigger<WalkStopCommand>>([this](const WalkStopCommand&){
                    if(!active){
                        emit(std::make_unique<FixedWalkFinished>());
                    } else {
                        NUClear::log("!!!!!!!!!!!!!!!!!!!!WARNING: Walk finised prematurely!!!!!!!!!!!!!!!!!!!!");
                    }
                });

				on<Trigger<FixedWalkCommand>, Options<Sync<FixedWalk>>, With<Sensors> >([this](const FixedWalkCommand& command, const Sensors& sensors){
	        		if(!active){
                        active = true;
	        			segmentStart = NUClear::clock::now();
	        			beginningOrientation = sensors.orientation;
                        emit(std::make_unique<WalkStartCommand>());
	        		}
	        		for(auto& segment: command.segments){
	        			walkSegments.push(segment);
	        		}
				});

            }

            std::unique_ptr<WalkCommand> FixedWalk::getWalkCommand(const FixedWalkCommand::WalkSegment& segment, NUClear::clock::duration t, const Sensors&){
             	double timeSeconds = std::chrono::duration_cast<std::chrono::seconds>(t).count();
            	arma::vec2 directionInOriginalCoords = (segment.curvePeriod != 0 ? utility::math::matrix::zRotationMatrix(2 * M_PI * timeSeconds / segment.curvePeriod, 2) : arma::eye(2,2) ) * segment.direction;
            	arma::vec2 direction =  arma::normalise(directionInOriginalCoords);
            	auto result = std::make_unique<WalkCommand>();
            	result->rotationalSpeed = segment.normalisedAngularVelocity;
            	result->velocity = segment.normalisedVelocity * direction;
            	return std::move(result);
            }


        }  // skills
    }  // behaviours
}  // modules
