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

#include "RobotModel.h"

#include <armadillo>
#include <nuclear>

#include "utility/math/angle.h"
#include "utility/math/matrix.h"
#include "utility/math/coordinates.h"
#include "messages/localisation/FieldObject.h"
#include "utility/localisation/transform.h"
#include "messages/input/Sensors.h"
#include "messages/input/ServoID.h"

using messages::input::Sensors;
using messages::input::ServoID;
using utility::localisation::transform::SphericalRobotObservation;
using utility::localisation::transform::WorldToRobotTransform;
using utility::math::matrix::rotationMatrix;
using utility::math::matrix::zRotationMatrix;
using utility::math::coordinates::cartesianToRadial;
using utility::math::coordinates::cartesianToSpherical;

namespace modules {
namespace localisation {
namespace robot {

arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
    const arma::vec::fixed<RobotModel::size>& state, double deltaT) {
    arma::vec::fixed<RobotModel::size> state_ = state;
    state_.rows(kX,kY) += deltaT * state.rows(kVX,kVY);
    return state_;
}

/// Return the predicted observation of an object at the given position
arma::vec RobotModel::predictedObservation(
    const arma::vec::fixed<RobotModel::size>& state,
    const arma::vec3& actual_position,
    const Sensors& sensors) {
    //Rewrite:
    arma::mat33 imuRotation = zRotationMatrix(state(kImuOffset));
    arma::vec3 robotHeading_world = imuRotation * arma::mat(sensors.orientation.t()).col(0);
    auto obs = SphericalRobotObservation(state.rows(kX, kY),
                                         robotHeading_world.rows(0,1),
                                         actual_position);
    return obs;
}

//Odometry
arma::vec RobotModel::predictedObservation(
    const arma::vec::fixed<RobotModel::size>& state, const Sensors& sensors) {    
    //Returns robot relative velocity
    arma::mat33 imuRotation = zRotationMatrix(state(kImuOffset));
    arma::vec3 robotHeading_world = imuRotation * arma::mat(sensors.orientation.t()).col(0);
    return WorldToRobotTransform(arma::vec2{0,0}, robotHeading_world.rows(0,1), state.rows(kVX, kVY));
}

// Angle between goals
arma::vec RobotModel::predictedObservation(
    const arma::vec::fixed<RobotModel::size>& state,
    const std::vector<arma::vec>& actual_positions) {

    arma::vec diff_1 = actual_positions[0].rows(0, 1) - state.rows(kX, kY);
    arma::vec diff_2 = actual_positions[1].rows(0, 1) - state.rows(kX, kY);
    arma::vec radial_1 = cartesianToRadial(diff_1);
    arma::vec radial_2 = cartesianToRadial(diff_2);

    auto angle_diff = utility::math::angle::difference(radial_1[1], radial_2[1]);

    return { std::abs(angle_diff) };
}

arma::vec RobotModel::observationDifference(const arma::vec& a,
                                            const arma::vec& b) {
    if (a.n_elem == 1) {
        return a - b;
    } if (a.n_elem == 2) {
        return a - b;
    } else {
        // Spherical coordinates
        arma::vec3 result = a - b;
        result(1) = utility::math::angle::normalizeAngle(result(1)) * cfg_.observationDifferenceBearingFactor;
        result(2) = utility::math::angle::normalizeAngle(result(2)) * cfg_.observationDifferenceElevationFactor;
        return result;
    }
}

arma::vec::fixed<RobotModel::size> RobotModel::limitState(
    const arma::vec::fixed<RobotModel::size>& state) {

    return state;
}

arma::mat::fixed<RobotModel::size, RobotModel::size> RobotModel::processNoise() {
    arma::mat noise = arma::eye(RobotModel::size, RobotModel::size);
    noise(kX, kX) *= cfg_.processNoisePositionFactor;
    noise(kY, kY) *= cfg_.processNoisePositionFactor;
    noise(kVX, kVX) *= cfg_.processNoiseVelocityFactor;
    noise(kVY, kVY) *= cfg_.processNoiseVelocityFactor;
    noise(kImuOffset, kImuOffset) *= cfg_.processNoiseHeadingFactor;
    return noise;
}

// arma::mat33 RobotModel::getRobotToWorldTransform(const arma::vec::fixed<RobotModel::size>& state){
//     arma::vec2 normed_heading = arma::normalise(state.rows(kHeadingX,kHeadingY));
//     arma::mat33 T;

//     T << normed_heading[0] << -normed_heading[1] << state[kX] << arma::endr
//       << normed_heading[1] <<  normed_heading[0] << state[kY] << arma::endr
//       <<                 0 <<                  0 <<         1;

//     return T;
// }

// arma::mat33 RobotModel::getWorldToRobotTransform(const arma::vec::fixed<RobotModel::size>& state){
//     arma::vec2 normed_heading = arma::normalise(state.rows(kHeadingX,kHeadingY));
//     arma::mat33 Tinverse;
//     Tinverse << normed_heading[0] <<  normed_heading[1] <<         0 << arma::endr
//              <<-normed_heading[1] <<  normed_heading[0] <<         0 << arma::endr
//              <<                 0 <<                  0 <<         1;

//     Tinverse.submat(0,2,1,2) = -Tinverse.submat(0,0,1,1) * arma::vec2({state[kX], state[kY]});
//     return Tinverse;
// }

}
}
}
