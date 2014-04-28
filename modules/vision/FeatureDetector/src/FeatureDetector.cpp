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

#include "FeatureDetector.h"

namespace modules {
    namespace vision {

            using messages::support::Configuration;
            using utility::configuration::ConfigurationNode;
        using messages::vision::ClassifiedImage;
        using messages::vision::COLOUR_CLASS;
            using messages::vision::ColourSegment;
        using messages::platform::darwin::DarwinSensors;
        using messages::vision::LookUpTable;

        using messages::input::Sensors;
        using messages::input::ServoID;


        FeatureDetector::FeatureDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)),
                                m_visionKinematics() { //, m_ballDetector(), m_goalDetector(), m_fieldPointDetector(), m_obstacleDetector() {

            // Load feature detector constants.
                    /*
            on<Trigger<Configuration<FeatureDetectorConfig>>>([this](const Configuration<FeatureDetectorConfig>& constants) {
                DETECT_LINES = constants.config["DETECT_LINES"];
                DETECT_GOALS = constants.config["DETECT_GOALS"];
                DETECT_BALLS = constants.config["DETECT_BALLS"];
                DETECT_OBSTACTLES = constants.config["DETECT_OBSTACTLES"];

                if (DETECT_LINES) {
                    m_detectLineObjects.enable();
                }

                else {
                    m_detectLineObjects.disable();
                }

                if (DETECT_GOALS) {
                    m_detectGoals.enable();
                }

                else {
                    m_detectGoals.disable();
                }

                if (DETECT_BALLS) {
                    m_detectBalls.enable();
                }

                else {
                    m_detectBalls.disable();
                }

                if (DETECT_OBSTACTLES) {
                    m_detectObstacles.enable();
                }

                else {
                    m_detectObstacles.disable();
                }
            });
                */

            on<Trigger<Configuration<VisionKinematicsConfig>>>([this](const Configuration<VisionKinematicsConfig>& constants) {
                m_visionKinematics.setParameters(constants.config["RADIAL_CORRECTION_COEFFICIENT"],                                            
                                                 constants.config["SCREEN_LOCATION_UNCERTAINTY_PIXELS"]);
            });

            // TODO: on<Trigger<Configuration<FieldPointDetectorConfig>>>().
            // TODO: on<Trigger<Configuration<ObstalceDetectorConfig>>>().
            // TODO: Ensure all config files are up to date and include constants for the appropriate subclasses.
            on<Trigger<Configuration<BallDetectorConfig>>>([this](const Configuration<BallDetectorConfig>& constants) {
                    DISTANCE_METHOD distanceMethod;
                    std::string BALL_DISTANCE_METHOD = constants.config["BALL_DISTANCE_METHOD"];

                    if (BALL_DISTANCE_METHOD.compare("WIDTH") == 0) {
                        distanceMethod = DISTANCE_METHOD::WIDTH;
                    }

                    else if (BALL_DISTANCE_METHOD.compare("D2P") == 0) {
                        distanceMethod = DISTANCE_METHOD::D2P;
                    }

                    else if (BALL_DISTANCE_METHOD.compare("AVERAGE") == 0) {
                        distanceMethod = DISTANCE_METHOD::AVERAGE;
                    }

                    else if (BALL_DISTANCE_METHOD.compare("LEAST") == 0) {
                        distanceMethod = DISTANCE_METHOD::LEAST;
                    }

                    else if (BALL_DISTANCE_METHOD.compare("ADAPTIVE") == 0) {
                        distanceMethod = DISTANCE_METHOD::ADAPTIVE;
                    }

                    else {
                        distanceMethod = DISTANCE_METHOD::WIDTH;
                    }


                    m_ballDetector.setParameters(constants.config["BALL_EDGE_THRESHOLD"],
                                                 constants.config["BALL_ORANGE_TOLERANCE"],
                                                 constants.config["BALL_MIN_PERCENT_ORANGE"],
                                                 constants.config["THROWOUT_ON_ABOVE_KIN_HOR_BALL"],
                                                 constants.config["MAX_DISTANCE_METHOD_DISCREPENCY_BALL"],
                                                 constants.config["THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL"],
                                                 constants.config["THROWOUT_SMALL_BALLS"],
                                                 constants.config["MIN_BALL_DIAMETER_PIXELS"],
                                                 constants.config["THROWOUT_DISTANT_BALLS"],
                                                 constants.config["MAX_BALL_DISTANCE"],
                                                 constants.config["BALL_WIDTH"],
                                                 distanceMethod,
                                                 constants.config["D2P_ADAPTIVE_THRESHOLD"]
                                                 );

            });

            on<Trigger<Configuration<GoalDetectorConfig>>>([this](const Configuration<GoalDetectorConfig>& constants) {
                    RANSAC_SELECTION_METHOD selectionMethod;
                    DISTANCE_METHOD distanceMethod;
                    std::string GOAL_DISTANCE_METHOD = constants.config["GOAL_DISTANCE_METHOD"];
                    std::string SELECTION_METHOD = constants.config["SELECTION_METHOD"];

                    if (GOAL_DISTANCE_METHOD.compare("WIDTH") == 0) {
                        distanceMethod = DISTANCE_METHOD::WIDTH;
                    }

                    else if (GOAL_DISTANCE_METHOD.compare("D2P") == 0) {
                        distanceMethod = DISTANCE_METHOD::D2P;
                    }

                    else if (GOAL_DISTANCE_METHOD.compare("AVERAGE") == 0) {
                        distanceMethod = DISTANCE_METHOD::AVERAGE;
                    }

                    else if (GOAL_DISTANCE_METHOD.compare("LEAST") == 0) {
                        distanceMethod = DISTANCE_METHOD::LEAST;
                    }
                    else if (GOAL_DISTANCE_METHOD.compare("ADAPTIVE") == 0) {
                        distanceMethod = DISTANCE_METHOD::ADAPTIVE;
                    }

                    else {
                        distanceMethod = DISTANCE_METHOD::WIDTH;
                    }

                    if (SELECTION_METHOD.compare("LARGEST_CONSENSUS") == 0) {
                        selectionMethod = RANSAC_SELECTION_METHOD::LargestConsensus;
                    }

                    else if (SELECTION_METHOD.compare("BEST_FITTING_CONSENSUS") == 0) {
                        selectionMethod = RANSAC_SELECTION_METHOD::BestFittingConsensus;
                    }

                    else {
                        selectionMethod = RANSAC_SELECTION_METHOD::LargestConsensus;
                    }


                    m_goalDetector.setParameters(constants.config["MINIMUM_POINTS"],
                                                 constants.config["MAX_ITERATIONS_PER_FITTING"],
                                                 constants.config["CONSENSUS_THRESHOLD"],
                                                 constants.config["MAX_FITTING_ATTEMPTS"],
                                                 constants.config["ANGLE_MARGIN"],
//                                                 KinematicsHorizon,
                                                 selectionMethod,
                                                 constants.config["RANSAC_MATCHING_TOLERANCE"],
                                                 constants.config["MIN_GOAL_SEPARATION"],
                                                 constants.config["GOAL_HEIGHT_TO_WIDTH_RATIO_MIN"],
                                                 constants.config["THROWOUT_SHORT_GOALS"],
                                                 constants.config["THROWOUT_NARROW_GOALS"],
                                                 constants.config["THROWOUT_ON_ABOVE_KIN_HOR_GOALS"],
                                                 constants.config["THROWOUT_DISTANT_GOALS"],
                                                 constants.config["MAX_GOAL_DISTANCE"],
                                                 constants.config["MIN_GOAL_HEIGHT"],
                                                 constants.config["MIN_GOAL_WIDTH"],
                                                 constants.config["GOAL_WIDTH"],
                                                 distanceMethod,
                                                 constants.config["EDGE_OF_SCREEN_MARGIN"],
                                                 constants.config["D2P_ADAPTIVE_THRESHOLD"]);

            });

            on<Trigger<Configuration<ObstacleDetectorConfig>>>([this](const Configuration<ObstacleDetectorConfig>& constants) {
                    m_obstacleDetector.setParameters(constants.config["MIN_DISTANCE_FROM_HORIZON"],
                                                 constants.config["VERTICAL_SCANLINE_SPACING"],
                                                 constants.config["MIN_CONSECUTIVE_POINTS"],
                                                 constants.config["MIN_COLOUR_THRESHOLD"],
                                                 constants.config["MAX_OTHER_COLOUR_THRESHOLD"],
                                                 constants.config["VER_THRESHOLD"],
                                                 constants.config["OBJECT_THRESHOLD_MULT"]);

            });

            on<Trigger<Configuration<CameraConfig>>>([this](const Configuration<CameraConfig>& config) {
                    arma::vec2 FOV, imageSize;

                    FOV[0] = config.config["FOV_X"];
                    FOV[1] = config.config["FOV_Y"];

                    imageSize[0] = config.config["imageWidth"];
                    imageSize[1] = config.config["imageHeight"];

                    m_visionKinematics.setCamParams(imageSize,FOV);
            });


            on<Trigger<Sensors>, Options<Sync<ObstacleDetectorConfig>, 
                                         Sync<BallDetectorConfig>, 
                                         Sync<GoalDetectorConfig>>>([this](const Sensors& sensors){
                arma::vec v = {0,0,0.35};
                m_visionKinematics.setSensors(sensors);
            });

            /*
            m_detectLineObjects = on<Trigger<ClassifiedImage>>([this](const ClassifiedImage& classifiedImage) {

            });

            */
            //m_detectGoals =


            on<Trigger<ClassifiedImage>, Options<Single, Sync<GoalDetectorConfig>>>([this](const ClassifiedImage& classifiedImage) {
                if (classifiedImage.matchedHorizontalSegments.count(messages::vision::GOAL_COLOUR) &&
                    classifiedImage.matchedVerticalSegments.count(messages::vision::GOAL_COLOUR)) {
                    emit(
                        m_goalDetector.run(m_visionKinematics,
                                           classifiedImage.matchedHorizontalSegments.at(messages::vision::GOAL_COLOUR),
                                           classifiedImage.matchedVerticalSegments.at(messages::vision::GOAL_COLOUR))
                        );
                }
            });

            //m_detectBalls =
            on<Trigger<ClassifiedImage>, Options<Single, Sync<BallDetectorConfig>>>([this](const ClassifiedImage& classifiedImage) {
                emit(
                    m_ballDetector.run(
                                        classifiedImage.matchedHorizontalSegments.at(messages::vision::BALL_COLOUR),
                                        classifiedImage.matchedVerticalSegments.at(messages::vision::BALL_COLOUR),
                                        classifiedImage.greenHorizonInterpolatedPoints,
                                        *(classifiedImage.image),
                                        *(classifiedImage.LUT),
                                        m_visionKinematics
                                       )
                );
            });

            //m_detectObstacles =
            on<Trigger<ClassifiedImage>, Options<Single, Sync<ObstacleDetectorConfig>>>([this](const ClassifiedImage& classifiedImage) {
                emit(
                    m_obstacleDetector.run(classifiedImage.greenHorizonInterpolatedPoints,
                                           *(classifiedImage.LUT),
                                           *(classifiedImage.image),
                                           classifiedImage.getAllMatchedSegments(messages::vision::TEAM_CYAN_COLOUR),
                                           classifiedImage.getAllMatchedSegments(messages::vision::TEAM_MAGENTA_COLOUR),
                                           m_visionKinematics
                                          )
                );
            });

        }
    }  // vision
}  // modules
