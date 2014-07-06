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

#include "ObstacleDetector.h"

#include <armadillo>
#include "messages/vision/ClassifiedImage.h"

namespace modules {
namespace vision {

    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;

    ObstacleDetector::ObstacleDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<ClassifiedImage<ObjectClass>>, Options<Single>>("Obstacle Detector", [this](const ClassifiedImage<ObjectClass>& image) {

            struct PointData {
                arma::ivec2 point;
                int direction;
                ObjectClass team;
            };

            std::vector<PointData> points;

            // Get all the segments that are relevant to finding an obstacle
            for(int i = 0; i < 6; ++i) {

                auto segments = i == 0 ? image.horizontalSegments.equal_range(ObjectClass::UNKNOWN)
                              : i == 1 ? image.verticalSegments.equal_range(ObjectClass::UNKNOWN)
                              : i == 2 ? image.horizontalSegments.equal_range(ObjectClass::CYAN_TEAM)
                              : i == 3 ? image.verticalSegments.equal_range(ObjectClass::CYAN_TEAM)
                              : i == 4 ? image.horizontalSegments.equal_range(ObjectClass::MAGENTA_TEAM)
                              :          image.verticalSegments.equal_range(ObjectClass::MAGENTA_TEAM);

                auto colour = i == 0 ? ObjectClass::UNKNOWN
                            : i == 1 ? ObjectClass::UNKNOWN
                            : i == 2 ? ObjectClass::CYAN_TEAM
                            : i == 3 ? ObjectClass::CYAN_TEAM
                            : i == 4 ? ObjectClass::MAGENTA_TEAM
                            :          ObjectClass::MAGENTA_TEAM;

                for(auto it = segments.first; it != segments.second; ++it) {

                    auto& start = it->second.start;
                    auto& end = it->second.end;

                    // Push back starts only if we are a horizontal segment
                    if(it->second.previous && image.visualHorizonAtPoint(start[0]) < start[1] && !(i % 2)) {
                        points.push_back({ start, 1, colour });
                    }

                    // Check if we have a subsequent segment
                    if(it->second.next && image.visualHorizonAtPoint(end[0]) < end[1]) {
                        points.push_back({ end, !(i % 2) ? -1 : 0, colour });
                    }
                }
            }

            // Sort our points by their x position
            std::sort(points.begin(), points.end(), [] (const PointData& a, const PointData& b) {
                return a.point[0] < b.point[0];
            });

            int counter = 0;
            std::vector<PointData>::iterator start;
            std::vector<PointData>::iterator end;
            for(auto it = points.begin(); it != points.end(); ++it) {
                // We add to the counter what our transition is
                counter += it->direction;

                if(counter == MINIMUM_SEGMENTS_FOR_OBSTACLE && points->direction == 1) {
                    // Start tracking our segments
                    start = it;
                }
                else if(counter == MINIMUM_SEGMENTS_FOR_OBSTACLE && points->direction == -1) {
                    // Build our obstacle by doing a hull of some sort from the highest point
                }

                // If we are at the boundry and we just added one
                // Start appending points

                // if we are at the boundry and we just subtracted one
                // Build the obstacle
            }

            //JAKE's Vision Kinematics for distance to obstacle:
            // arma::vec2 p1 = imageToScreen(obstacleBaseCentreImage, { double(image.dimensions[0]), double(image.dimensions[1]) });

            // arma::vec3 obstacleBaseGroundProj = projectCamToGroundPlane(p1, sensors.orientationCamToGround);
            // //Testing (not done yet - TODO: TEST AND REMOVE THIS NOTE)
            // std::cout << "orientationCamToGround\n" << sensors.orientationCamToGround << std::endl;
            // std::cout << "D2P obstacle: " << obstacleBaseGroundProj.t() << std::endl;
            // emit(graph("D2P Obstacle", obstacleBaseGroundProj[0], obstacleBaseGroundProj[1]));


        });

    }

}
}

