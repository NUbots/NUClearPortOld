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

#include "SensorFilter.h"

#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/Sensors.h"
#include "messages/input/CameraParameters.h"
#include "messages/support/Configuration.h"
#include "utility/nubugger/NUgraph.h"
#include "utility/math/matrix.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/RobotModels.h"

namespace modules {
    namespace platform {
        namespace darwin {

            using messages::support::Configuration;
            using messages::platform::darwin::DarwinSensors;
            using messages::input::Sensors;
            using messages::input::CameraParameters;
            using utility::nubugger::graph;
            using messages::input::ServoID;
            using utility::motion::kinematics::calculateAllPositions;
            using utility::motion::kinematics::DarwinModel;
            using utility::motion::kinematics::calculateCentreOfMass;
            using utility::motion::kinematics::Side;
            using utility::math::matrix::orthonormal44Inverse;
            using utility::math::matrix::quaternionToRotationMatrix;
            using utility::math::kalman::IMUModel;

            SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , orientationFilter(arma::vec({0,0,0,1,0,0,0}))
            , velocityFilter(arma::vec3({0,0,0})) {

                on<Trigger<Configuration<SensorFilter>>>([this](const Configuration<SensorFilter>& file){
                    DEFAULT_NOISE_GAIN = file.config["DEFAULT_NOISE_GAIN"].as<double>();
                    HIGH_NOISE_THRESHOLD = file.config["HIGH_NOISE_THRESHOLD"].as<double>();
                    HIGH_NOISE_GAIN = file.config["HIGH_NOISE_GAIN"].as<double>();
                    LOW_NOISE_THRESHOLD = file.config["LOW_NOISE_THRESHOLD"].as<double>();

                    SUPPORT_FOOT_FSR_THRESHOLD = file.config["SUPPORT_FOOT_FSR_THRESHOLD"].as<double>();
                    REQUIRED_NUMBER_OF_FSRS = file.config["REQUIRED_NUMBER_OF_FSRS"].as<int>();

                    orientationFilter.model.processNoiseDiagonal = arma::ones(orientationFilter.model.size);
                    orientationFilter.model.processNoiseDiagonal.rows(orientationFilter.model.QW,orientationFilter.model.QZ) *= file["IMU_POSITION_PROCESS_NOISE"].as<double>();
                    orientationFilter.model.processNoiseDiagonal.rows(orientationFilter.model.VX,orientationFilter.model.VZ) *= file["IMU_VELOCITY_PROCESS_NOISE"].as<double>();
                    NUClear::log("ProcessNoise Set: \n", orientationFilter.model.processNoiseDiagonal.t());

                    MEASUREMENT_NOISE_ACCELEROMETER = arma::eye(3,3) * file["MEASUREMENT_NOISE_ACCELEROMETER"].as<double>();
                    MEASUREMENT_NOISE_GYROSCOPE = arma::eye(3,3) * file["MEASUREMENT_NOISE_GYROSCOPE"].as<double>();
                });

                on< Trigger<DarwinSensors>
                  , With<Optional<Sensors>>
                  , With<CameraParameters>
                  , Options<Single>>([this](const DarwinSensors& input,
                                            const std::shared_ptr<const Sensors>& previousSensors,
                                            const CameraParameters& cameraParameters) {

                    auto sensors = std::make_unique<Sensors>();

                    // Set our timestamp to when the data was read
                    sensors->timestamp = input.timestamp;

                    // This checks for an error on the CM730 and reports it
                    if (input.cm730ErrorFlags != DarwinSensors::Error::OK) {
                        std::stringstream s;
                        s << "Error on CM730:";

                        if(input.cm730ErrorFlags & DarwinSensors::Error::INPUT_VOLTAGE) {
                            s << " Input Voltage ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::ANGLE_LIMIT) {
                            s << " Angle Limit ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::OVERHEATING) {
                            s << " Overheating ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::OVERLOAD) {
                            s << " Overloaded ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::INSTRUCTION) {
                            s << " Bad Instruction ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::CORRUPT_DATA) {
                            s << " Corrupt Data ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::TIMEOUT) {
                            s << " Timeout ";
                        }

                        NUClear::log<NUClear::WARN>(s.str());
                    }

                    // Output errors on the FSRs
                    if (input.fsr.left.errorFlags != DarwinSensors::Error::OK) {
                        std::stringstream s;
                        s << "Error on Left FSR:";

                        if(input.cm730ErrorFlags & DarwinSensors::Error::INPUT_VOLTAGE) {
                            s << " Input Voltage ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::ANGLE_LIMIT) {
                            s << " Angle Limit ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::OVERHEATING) {
                            s << " Overheating ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::OVERLOAD) {
                            s << " Overloaded ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::INSTRUCTION) {
                            s << " Bad Instruction ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::CORRUPT_DATA) {
                            s << " Corrupt Data ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::TIMEOUT) {
                            s << " Timeout ";
                        }

                        NUClear::log<NUClear::WARN>(s.str());
                    }

                    if (input.fsr.right.errorFlags != DarwinSensors::Error::OK) {
                        std::stringstream s;
                        s << "Error on Right FSR:";

                        if(input.cm730ErrorFlags & DarwinSensors::Error::INPUT_VOLTAGE) {
                            s << " Input Voltage ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::ANGLE_LIMIT) {
                            s << " Angle Limit ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::OVERHEATING) {
                            s << " Overheating ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::OVERLOAD) {
                            s << " Overloaded ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::INSTRUCTION) {
                            s << " Bad Instruction ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::CORRUPT_DATA) {
                            s << " Corrupt Data ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::TIMEOUT) {
                            s << " Timeout ";
                        }

                        NUClear::log<NUClear::WARN>(s.str());
                    }

                    // Read through all of our sensors
                    for(uint i = 0; i < 20; ++i) {
                        auto& original = input.servo[i];
                        auto& error = original.errorFlags;

                        // Check for an error on the servo and report it
                        while(error != DarwinSensors::Error::OK) {
                            std::stringstream s;
                            s << "Error on Servo " << (i + 1) << " (" << messages::input::stringFromId(ServoID(i)) << "):";

                            if(error & DarwinSensors::Error::INPUT_VOLTAGE) {
                                s << " Input Voltage - " << original.voltage;
                            }
                            if(error & DarwinSensors::Error::ANGLE_LIMIT) {
                                s << " Angle Limit - " << original.presentPosition;
                            }
                            if(error & DarwinSensors::Error::OVERHEATING) {
                                s << " Overheating - " << original.temperature;
                            }
                            if(error & DarwinSensors::Error::OVERLOAD) {
                                s << " Overloaded - " << original.load;
                            }
                            if(error & DarwinSensors::Error::INSTRUCTION) {
                                s << " Bad Instruction ";
                            }
                            if(error & DarwinSensors::Error::CORRUPT_DATA) {
                                s << " Corrupt Data ";
                                break;
                            }
                            if(error & DarwinSensors::Error::TIMEOUT) {
                                s << " Timeout ";
                            }

                            NUClear::log<NUClear::WARN>(s.str());
                            break;
                        }

                        // Add the sensor values to the system properly
                        sensors->servos.push_back({
                            original.errorFlags,
                            static_cast<ServoID>(i),
                            original.torqueEnabled,
                            original.pGain,
                            original.iGain,
                            original.dGain,
                            original.goalPosition,
                            original.movingSpeed,
                            original.presentPosition,
                            original.presentSpeed,
                            original.load,
                            original.voltage,
                            float(original.temperature)
                        });
                    }

                    // If we have a previous sensors and our cm730 has errors then reuse our last sensor value
                    if(previousSensors && (input.cm730ErrorFlags)) {
                        sensors->accelerometer = previousSensors->accelerometer;
                    }
                    else {
                        sensors->accelerometer = {-input.accelerometer.y, input.accelerometer.x, -input.accelerometer.z};
                    }

                    // If we have a previous sensors and our cm730 has errors then reuse our last sensor value
                    if(previousSensors && (input.cm730ErrorFlags || arma::norm(arma::vec({input.gyroscope.x, input.gyroscope.y, input.gyroscope.z}), 2) > 4 * M_PI)) {
                        // NUClear::log("Bad gyroscope value", arma::norm(arma::vec({input.gyroscope.x, input.gyroscope.y, input.gyroscope.z}), 2));
                        sensors->gyroscope = previousSensors->gyroscope;
                    }
                    else {
                        sensors->gyroscope = {-input.gyroscope.x, -input.gyroscope.y, input.gyroscope.z};
                    }

                    /************************************************
                     *                 Orientation                  *
                     ************************************************/

                    // Calculate our time offset from the last read
                    double deltaT = ((previousSensors ? previousSensors->timestamp : input.timestamp) - input.timestamp).count() / double(NUClear::clock::period::den);

                    orientationFilter.timeUpdate(deltaT);

                    orientationFilter.measurementUpdate(sensors->accelerometer, MEASUREMENT_NOISE_ACCELEROMETER, IMUModel::MeasurementType::ACCELEROMETER());
                    orientationFilter.measurementUpdate(sensors->gyroscope,     MEASUREMENT_NOISE_GYROSCOPE, IMUModel::MeasurementType::GYROSCOPE());

                    // Gives us the quaternion representation
                    arma::vec o = orientationFilter.get();
                    //Map from robot to world coordinates
                    sensors->orientation = quaternionToRotationMatrix(o.rows(orientationFilter.model.QW, orientationFilter.model.QZ));

                    // sensors->orientation.col(2) = -orientation.rows(0,2);
                    // sensors->orientation.col(0) = orientation.rows(3,5);
                    // sensors->orientation.col(1) = arma::cross(sensors->orientation.col(2), sensors->orientation.col(0));

                    /************************************************
                     *                  Kinematics                  *
                     ************************************************/
                    sensors->forwardKinematics = calculateAllPositions<DarwinModel>(*sensors);

                    /************************************************
                     *                   Odometry                   *
                     ************************************************/

                    //Check support foot:
                    sensors->leftFootDown = false;
                    sensors->rightFootDown = false;

                    // int zeroSensorsLeft = (input.fsr.left.fsr1 == 0) + (input.fsr.left.fsr2 == 0) + (input.fsr.left.fsr3 == 0) + (input.fsr.left.fsr4 == 0);
                    // int zeroSensorsRight = (input.fsr.right.fsr1 == 0) + (input.fsr.right.fsr2 == 0) + (input.fsr.right.fsr3 == 0) + (input.fsr.right.fsr4 == 0);

                    // if(input.fsr.left.fsr1 + input.fsr.left.fsr2 + input.fsr.left.fsr3 + input.fsr.left.fsr4 > SUPPORT_FOOT_FSR_THRESHOLD && zeroSensorsLeft <= 4 - REQUIRED_NUMBER_OF_FSRS){
                    //     sensors->leftFootDown = true;
                    // }
                    // if(input.fsr.right.fsr1 + input.fsr.right.fsr2 + input.fsr.right.fsr3 + input.fsr.right.fsr4 > SUPPORT_FOOT_FSR_THRESHOLD && zeroSensorsRight <= 4 - REQUIRED_NUMBER_OF_FSRS){
                    //     sensors->rightFootDown = true;
                    // }

                    if(!std::isnan(input.fsr.left.centreX) && !std::isnan(input.fsr.left.centreY)) {
                        // Left foot is on the ground?
                        sensors->leftFootDown = true;
                    }
                    if(!std::isnan(input.fsr.right.centreX) && !std::isnan(input.fsr.right.centreY)) {
                        // Right foot is on the ground?
                        sensors->rightFootDown = true;
                    }

                    // if(previousSensors && (!sensors->leftFootDown && !sensors->rightFootDown )) {
                    //     //std::cout << "No feet down!" << std::endl;
                    //     sensors->leftFootDown = previousSensors->leftFootDown;
                    //     sensors->rightFootDown = previousSensors->rightFootDown;
                    // }

                    sensors->odometry = arma::eye(4,4);
                    // // Kinematics odometry
                    // arma::mat44 odometryRightFoot = arma::eye(4,4);
                    // arma::mat44 odometryLeftFoot = arma::eye(4,4);
                    // if(previousSensors){
                    //     //NOTE: calculateOdometryMatrix requires sensors->forwardKinematics to be calculated before calling
                    //     odometryLeftFoot = calculateOdometryMatrix(*sensors, *previousSensors, Side::LEFT);
                    //     odometryRightFoot = calculateOdometryMatrix(*sensors, *previousSensors, Side::RIGHT);
                    // }


                    // if(sensors->leftFootDown || sensors->rightFootDown){
                    //     sensors->odometry.submat(0,3,2,3) = (odometryLeftFoot.submat(0,3,2,3) * sensors->leftFootDown + odometryLeftFoot.submat(0,3,2,3) * sensors->rightFootDown)
                    //                                         / (sensors->leftFootDown + sensors->rightFootDown);
                    //     if(sensors->leftFootDown && sensors->rightFootDown){
                    //         sensors->odometry.submat(0,0,2,2) = odometryLeftFoot.submat(0,0,2,2);
                    //     } else {
                    //         sensors->odometry.submat(0,0,2,2) = odometryLeftFoot.submat(0,0,2,2) * sensors->leftFootDown + odometryRightFoot.submat(0,0,2,2) * sensors->rightFootDown;
                    //     }
                    // }

                    if(previousSensors){
                        if(sensors->leftFootDown || sensors->rightFootDown){
                            arma::vec3 measuredTorsoFromLeftFoot = -sensors->forwardKinematics.at(ServoID::L_ANKLE_ROLL).submat(0,0,2,2).t() * sensors->forwardKinematics.at(ServoID::L_ANKLE_ROLL).col(3).rows(0,2);
                            arma::vec3 measuredTorsoFromRightFoot = -sensors->forwardKinematics.at(ServoID::R_ANKLE_ROLL).submat(0,0,2,2).t() * sensors->forwardKinematics.at(ServoID::R_ANKLE_ROLL).col(3).rows(0,2);

                            arma::vec3 previousMeasuredTorsoFromLeftFoot = -previousSensors->forwardKinematics.at(ServoID::L_ANKLE_ROLL).submat(0,0,2,2).t() * previousSensors->forwardKinematics.at(ServoID::L_ANKLE_ROLL).col(3).rows(0,2);
                            arma::vec3 previousMeasuredTorsoFromRightFoot = -previousSensors->forwardKinematics.at(ServoID::R_ANKLE_ROLL).submat(0,0,2,2).t() * previousSensors->forwardKinematics.at(ServoID::R_ANKLE_ROLL).col(3).rows(0,2);

                            arma::vec3 torsoVelFromLeftFoot =  -(measuredTorsoFromLeftFoot - previousMeasuredTorsoFromLeftFoot);//negate hack
                            arma::vec3 torsoVelFromRightFoot =  -(measuredTorsoFromRightFoot - previousMeasuredTorsoFromRightFoot);

                            arma::vec3 averageVelocity = (torsoVelFromLeftFoot * static_cast<int>(sensors->leftFootDown) + torsoVelFromRightFoot * static_cast<int>(sensors->rightFootDown))/(static_cast<int>(sensors->rightFootDown) + static_cast<int>(sensors->leftFootDown));
                            sensors->odometry.submat(0,3,2,3) = averageVelocity;
                        }

                        // Gyro based odometry for orientation
                        sensors->odometry.submat(0,0,2,2) =  previousSensors->orientation.t() * sensors->orientation;
                    }

                    if(sensors->leftFootDown){
                        sensors->bodyCentreHeight = -sensors->forwardKinematics[ServoID::L_ANKLE_ROLL](2,3);
                    } else if(sensors->rightFootDown){
                        sensors->bodyCentreHeight = -sensors->forwardKinematics[ServoID::R_ANKLE_ROLL](2,3);
                    } else {
                        sensors->bodyCentreHeight = 0;
                    }


                    /************************************************
                     *                  Mass Model                  *
                     ************************************************/
                    //LOOKOUT!!!! ARRAYOPS_MEAT
                    arma::vec4 COM = calculateCentreOfMass<DarwinModel>(sensors->forwardKinematics, true);
                    sensors->centreOfMass = {COM[0],COM[1], COM[2], COM[3]};
                    //END MASS MODEL


                    /************************************************
                     *                  Kinematics Horizon          *
                     ************************************************/


                    sensors->orientationCamToGround = utility::motion::kinematics::calculateCamToGround(sensors->forwardKinematics[ServoID::HEAD_PITCH],
                                                                                                        sensors->orientation.submat(0,2,2,2),
                                                                                                        sensors->bodyCentreHeight);
                    sensors->orientationHorizon = utility::motion::kinematics::calculateHorizon(sensors->orientationCamToGround.submat(0,0,2,2).t(),cameraParameters.focalLengthPixels);


                    if(sensors->leftFootDown) {
                        sensors->kinematicsCamToGround = utility::motion::kinematics::calculateCamToGround(sensors->forwardKinematics[ServoID::HEAD_PITCH],
                                                                                                           sensors->forwardKinematics[ServoID::L_ANKLE_ROLL].submat(0,2,2,2),
                                                                                                           sensors->bodyCentreHeight);
                    } else if (sensors->rightFootDown) {
                        sensors->kinematicsCamToGround = utility::motion::kinematics::calculateCamToGround(sensors->forwardKinematics[ServoID::HEAD_PITCH],
                                                                                                           sensors->forwardKinematics[ServoID::R_ANKLE_ROLL].submat(0,2,2,2),
                                                                                                           sensors->bodyCentreHeight);
                    }
                    else {
                        sensors->kinematicsCamToGround = sensors->orientationCamToGround;
                    }
                    // std::cout << "sensors->kinematicsCamToGround\n" << sensors->kinematicsCamToGround << std::endl;
                    // std::cout << "sensors->orientationCamToGround\n" << sensors->orientationCamToGround << std::endl;
                    // std::cout << "sensors->bodyCentreHeight\n" << sensors->bodyCentreHeight << std::endl;

                    sensors->kinematicsHorizon = utility::motion::kinematics::calculateHorizon(sensors->kinematicsCamToGround.submat(0,0,2,2).t(), cameraParameters.focalLengthPixels);

                    /*emit(graph("Filtered Gravity Vector",
                            float(orientation[0]*9.807),
                            float(orientation[1]*9.807),
                            float(orientation[2]*9.807)
                        ));
                     emit(graph("Filtered Forward Vector",
                            float(orientation[3]),
                            float(orientation[4]),
                            float(orientation[5])
                        ));
                    emit(graph("Orientation Quality", quality
                        ));
                    emit(graph("Difference from gravity", normAcc
                        ));
                    emit(graph("Gyro Filtered", sensors->gyroscope[0],sensors->gyroscope[1], sensors->gyroscope[2]
                        ));*/

                        integratedOdometry += sensors->odometry.submat(0,3,1,3);

                    /*emit(graph("LFoot Down", sensors->leftFootDown
                        ));
                    emit(graph("RFoot Down", sensors->rightFootDown
                        ));
                    emit(graph("Torso Velocity (vx,vy,vz)", sensors->odometry(0,3), sensors->odometry(1,3), sensors->odometry(2,3)
                        ));
                    emit(graph("Integrated Odometry", integratedOdometry[0], integratedOdometry[1]
                        ));
                    emit(graph("COM", sensors->centreOfMass[0], sensors->centreOfMass[1], sensors->centreOfMass[2], sensors->centreOfMass[3]
                        ));*/

                    emit(std::move(sensors));
                });
            }

            arma::mat44 SensorFilter::calculateOdometryMatrix(
                const messages::input::Sensors& sensors,
                const messages::input::Sensors& previousSensors,
                utility::motion::kinematics::Side side) {
                    arma::mat44 bodyFromAnkleInitialInverse, bodyFromAnkleFinal;
                    if(side == Side::LEFT){
                        bodyFromAnkleInitialInverse = previousSensors.forwardKinematics.at(ServoID::L_ANKLE_ROLL);   //Double Inverse
                        bodyFromAnkleFinal = orthonormal44Inverse(sensors.forwardKinematics.at(ServoID::L_ANKLE_ROLL));
                    } else {
                        bodyFromAnkleInitialInverse = previousSensors.forwardKinematics.at(ServoID::R_ANKLE_ROLL);   //Double Inverse
                        bodyFromAnkleFinal = orthonormal44Inverse(sensors.forwardKinematics.at(ServoID::R_ANKLE_ROLL));
                    }
                    return  bodyFromAnkleInitialInverse * bodyFromAnkleFinal;
            }


        }  // darwin
    }  // platform
}  // modules
