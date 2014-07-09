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

#ifndef MODULES_LOCALISATION_ROBOTMODEL_H
#define MODULES_LOCALISATION_ROBOTMODEL_H

#include <armadillo>
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"


namespace modules {
namespace localisation {
namespace robot {
    // Number of dimensions
    // The state consists of 3 components:
    //    1. The x position on the field
    //    2. The y position on the field
    //    3. The robot's heading (in radians)
    enum RobotModelStateComponents : int {
        kX = 0,
        kY = 1,
        kHeading = 2,
        // kHeadingX = 2,
        // kHeadingY = 3,
    };

    enum class MeasurementType {
        kBRGoalMeasurement,
        kBLGoalMeasurement,
        kLandmarkMeasurement,
        kAngleBetweenLandmarksMeasurement,
    };

    class RobotModel {
    public:
        static constexpr size_t size = 3;

        RobotModel() {} // empty constructor

        arma::vec::fixed<RobotModel::size> timeUpdate(
            const arma::vec::fixed<RobotModel::size>& state, double deltaT);

        // arma::vec predictedObservation(
        //     const arma::vec::fixed<RobotModel::size>& state,
        //     const arma::vec& actual_position);

        arma::vec predictedObservation(
            const arma::vec::fixed<RobotModel::size>& state,
            const arma::vec3& actual_position);

        arma::vec predictedObservation(
            const arma::vec::fixed<RobotModel::size>& state,
            const std::vector<arma::vec>& actual_positions);

        arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

        arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

        arma::mat::fixed<size, size> processNoise();

        double processNoiseFactor = 1e-3;

        arma::mat33 getRobotToWorldTransform(const arma::vec::fixed<RobotModel::size>& state);

        arma::mat33 getWorldToRobotTransform(const arma::vec::fixed<RobotModel::size>& state);
    };
}
}
}
#endif
