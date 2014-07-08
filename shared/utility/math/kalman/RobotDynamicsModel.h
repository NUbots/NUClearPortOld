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

#ifndef UTILITY_MATH_KALMAN_IMUMODEL_H
#define UTILITY_MATH_KALMAN_IMUMODEL_H

/* Inertial Motion Unit*/
#include <armadillo>

namespace utility {
    namespace math {
        namespace kalman {

            class RobotDynamicsModel {
            public:

                static constexpr double G = -9.80665;

                // The size of our model
                static constexpr size_t size = 13;

                // The indicies for our vector
                static constexpr uint QW = 0;   // Our quaternion rotation w
                static constexpr uint QX = 1;   // Our quaternion rotation x
                static constexpr uint QY = 2;   // Our quaternion rotation y
                static constexpr uint QZ = 3;   // Our quaternion rotation z
                static constexpr uint WX = 4;   // Our rotational velocity around x
                static constexpr uint WY = 5;   // Our rotational velocity around y
                static constexpr uint WZ = 6;   // Our rotational velcoity around z
                static constexpr uint PX = 7;   // Our position in X
                static constexpr uint PY = 8;   // Our position in Y
                static constexpr uint PZ = 9;   // Our position in Z
                static constexpr uint VX = 10;  // Our translational velocity in X
                static constexpr uint VY = 11;  // Our translational velocity in Y
                static constexpr uint VZ = 12;  // Our translational velocity in Z
                static constexpr uint AX = 13;  // Our translational acceleration in X
                static constexpr uint AY = 14;  // Our translational acceleration in Y
                static constexpr uint AZ = 15;  // Our translational acceleration in Z

                struct MeasurementType {
                    struct GYROSCOPE {};
                    struct ACCELEROMETER {};
                    struct KINEMATICS_DOWN {};
                    struct TORSO_VELOCITY {};
                };

                arma::vec processNoiseDiagonal;


                IMUModel() {} // empty constructor

                arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double deltaT);

                arma::vec3 predictedObservation(const arma::vec::fixed<size>& state);
                arma::vec3 predictedObservation(const arma::vec::fixed<size>& state, const MeasurementType::ACCELEROMETER&);
                arma::vec3 predictedObservation(const arma::vec::fixed<size>& state, const MeasurementType::GYROSCOPE&);
                arma::vec3 predictedObservation(const arma::vec::fixed<size>& state, const MeasurementType::KINEMATICS_DOWN&);
                arma::vec3 predictedObservation(const arma::vec::fixed<size>& state, const MeasurementType::TORSO_VELOCITY&);

                arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

                arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

                arma::mat::fixed<size, size> processNoise();
            };

        }
    }
}
#endif
