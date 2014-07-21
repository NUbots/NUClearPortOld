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

#include "MockRobot.h"
#include <nuclear>
#include "utility/math/angle.h"
#include "utility/math/matrix.h"
#include "utility/math/coordinates.h"
#include "utility/nubugger/NUgraph.h"
#include "utility/localisation/transform.h"
#include "messages/vision/VisionObjects.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "utility/localisation/transform.h"
#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"
#include "messages/input/Sensors.h"
#include "messages/input/ServoID.h"
#include "messages/input/gameevents/GameEvents.h"
#include "messages/behaviour/LookStrategy.h"

namespace modules {
    namespace behaviour {
        namespace strategy {

            using messages::input::Sensors;
            using messages::input::ServoID;
            using utility::math::matrix::rotationMatrix;
            using utility::math::angle::normalizeAngle;
            using utility::math::angle::vectorToBearing;
            using utility::math::angle::bearingToUnitVector;
            using utility::math::coordinates::cartesianToSpherical;
            using utility::localisation::transform::SphericalRobotObservation;
            using utility::localisation::transform::WorldToRobotTransform;
            using utility::localisation::transform::RobotToWorldTransform;
            using utility::nubugger::graph;
            using messages::support::Configuration;
            using messages::support::FieldDescription;
            using modules::behaviour::strategy::MockRobotConfig;
            using messages::localisation::Mock;
            using messages::input::gameevents::GameState;
            using messages::input::gameevents::Phase;
            using messages::input::gameevents::Mode;
            using messages::input::gameevents::PenaltyReason;
            using messages::behaviour::LookAtAngle;
            using messages::behaviour::LookAtPosition;
            using messages::motion::KickCommand;
            using messages::motion::KickFinished;
            using messages::behaviour::LimbID;

            double triangle_wave(double t, double period) {
                auto a = period; // / 2.0;
                auto k = t / a;
                return 2.0 * std::abs(2.0 * (k - std::floor(k + 0.5))) - 1.0;
            }

            double sawtooth_wave(double t, double period) {
                return 2.0 * std::fmod(t / period, 1.0) - 1.0;
            }

            double square_wave(double t, double period) {
                return std::copysign(1.0, sawtooth_wave(t, period));
            }

            double sine_wave(double t, double period) {
                return std::sin((2.0 * M_PI * t) / period);
            }

            double absolute_time() {
                auto now = NUClear::clock::now();
                auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
                double ms = static_cast<double>(ms_since_epoch - 1393322147502L);
                double t = ms / 1000.0;
                return t;
            }

            // Copied from KickScript.cpp
            int getDirectionalQuadrant(float x, float y) {

                    // These represent 4 directions of looking, see https://www.desmos.com/calculator/mm8cnsnpdt for a graph of the 4 quadrants
                    // Note that x is forward in relation to the robot so the forward quadrant is x >= |y|
                    return x >=  std::abs(y) ? 0  // forward
                         : y >=  std::abs(x) ? 1  // left
                         : x <= -std::abs(y) ? 2  // backward
                         :                     3; // right
            }

            void MockRobot::UpdateConfiguration(const messages::support::Configuration<MockRobotConfig>& config) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                cfg_.simulate_vision = config["SimulateVision"].as<bool>();
                cfg_.simulate_goal_observations = config["SimulateGoalObservations"].as<bool>();
                cfg_.simulate_ball_observations = config["SimulateBallObservations"].as<bool>();
                cfg_.simulate_odometry = config["SimulateOdometry"].as<bool>();
                cfg_.simulate_robot_movement = config["SimulateRobotMovement"].as<bool>();
                cfg_.simulate_robot_walking = config["SimulateRobotWalking"].as<bool>();
                cfg_.simulate_game_controller = config["SimulateGameController"].as<bool>();
                cfg_.robot_movement_path_period = config["RobotMovementPathPeriod"].as<double>();
                cfg_.simulate_ball_movement = config["SimulateBallMovement"].as<bool>();
                cfg_.simulate_ball_velocity_decay = config["SimulateBallVelocityDecay"].as<bool>();
                cfg_.ball_velocity_decay = config["BallVelocityDecay"].as<double>();
                cfg_.initial_kick_velocity = config["InitialKickVelocity"].as<double>();
                cfg_.emit_robot_fieldobjects = config["EmitRobotFieldobjects"].as<bool>();
                cfg_.emit_ball_fieldobjects = config["EmitBallFieldobjects"].as<bool>();
                cfg_.robot_imu_drift_period = config["RobotImuDriftPeriod"].as<double>();
                cfg_.observe_left_goal = config["ObserveLeftGoal"].as<bool>();
                cfg_.observe_right_goal = config["ObserveRightGoal"].as<bool>();
                cfg_.distinguish_left_and_right_goals = config["DistinguishLeftAndRightGoals"].as<bool>();
                cfg_.emit_localisation_ball_vector = config["EmitLocalisationBallVector"].as<bool>();

                // Game Controller
                cfg_.gc_first_half = config["GCFirstHalf"].as<bool>();
                cfg_.gc_kicked_out_by_us = config["GCKickedOutByUs"].as<bool>();
                cfg_.gc_our_kick_off = config["GCOurKickOff"].as<bool>();
                cfg_.gc_team_id = config["GCTeamID"].as<int>();
                cfg_.gc_opponent_id = config["GCOpponentID"].as<int>();
                cfg_.gc_mode = config["GCMode"].as<int>();
                cfg_.gc_phase = config["GCPhase"].as<int>();
                cfg_.gc_penalty_reason = config["GCPenaltyReason"].as<int>();
                
                // Look strategies.
                // Pan speeds.
                cfg_.fast_speed = config["FastSpeed"].as<double>();
                cfg_.slow_speed = config["SlowSpeed"].as<double>();

                // Head limits.
                cfg_.min_yaw = config["MinYaw"].as<double>();
                cfg_.max_yaw = config["MaxYaw"].as<double>();
                cfg_.min_pitch = config["MinPitch"].as<double>();
                cfg_.max_pitch = config["MaxPitch"].as<double>();
                cfg_.screen_padding = config["ScreenPadding"].as<double>();
                cfg_.distance_threshold = config["DistanceThreshold"].as<double>();

                // Camera parameters.
                cfg_.camera_height = config["CameraHeight"].as<double>();
                cfg_.FOV = {config["FOV_X"].as<double>(), config["FOV_Y"].as<double>()};
            }

            MockRobot::MockRobot(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)) {

                on<Trigger<FieldDescription>>("FieldDescription Update", [this](const FieldDescription& desc) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                       field_description_ = std::make_shared<FieldDescription>(desc);
                });

                on<Trigger<Configuration<MockRobotConfig>>>("MockRobotConfig Update", [this](const Configuration<MockRobotConfig>& config) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    UpdateConfiguration(config);
                });

                // Update robot position
                on<Trigger<Every<10, std::chrono::milliseconds>>>("Mock Robot motion", [this](const time_t&) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    if (!cfg_.simulate_robot_movement) {
                        //robot_velocity_ = { 0, 0 };
                        return;
                    }

//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    auto t = absolute_time();
                    double period = cfg_.robot_movement_path_period;
                    double x_amp = 3;
                    double y_amp = 2;

                    arma::vec old_pos = robot_position_;

//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    auto wave1 = triangle_wave(t, period);
                    auto wave2 = triangle_wave(t + (period / 4.0), period);
                    // auto wave1 = sine_wave(t, period);
                    // auto wave2 = sine_wave(t + (period / 4.0), period);
                    robot_position_ = { wave1 * x_amp, wave2 * y_amp };

//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    arma::vec diff = robot_position_ - old_pos;

                    robot_heading_ = vectorToBearing(diff);
                    robot_velocity_ = robot_heading_ / 100.0;

//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;

                    double imu_period = cfg_.robot_imu_drift_period;
                    world_imu_direction = { std::cos(2 * M_PI * t / imu_period), std::sin(2 * M_PI * t / imu_period) };
                });

                // Simulate robot walking
                on<Trigger<Every<10, std::chrono::milliseconds>>, With<Optional<messages::motion::WalkCommand>>, Options<Sync<MockRobot>>>("Mock Robot walking", [this](const time_t&, const std::shared_ptr<const messages::motion::WalkCommand>& walk) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    if (!cfg_.simulate_robot_walking) {
                        return;
                    }

                    auto t = absolute_time();

                    // Update position
                    if (walk != NULL) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                        std::cerr << __LINE__ << std::endl;
                        robot_position_[0] += (walk->velocity[0]*cos(robot_heading_) - walk->velocity[1]*sin(robot_heading_)) * 15;
                        std::cerr << __LINE__ << std::endl;
                        robot_position_[1] += (walk->velocity[0]*sin(robot_heading_) + walk->velocity[1]*cos(robot_heading_)) * 15;
                        std::cerr << __LINE__ << std::endl;
                        robot_heading_ += (walk->rotationalSpeed) * 10;
                        std::cerr << __LINE__ << std::endl;
                    }

                    double imu_period = cfg_.robot_imu_drift_period;
                    world_imu_direction = { std::cos(2 * M_PI * t / imu_period), std::sin(2 * M_PI * t / imu_period) };
                });

                // Simulate game controller
                on<Trigger<Every<100, std::chrono::milliseconds>>>("Mock Game Controller", [this](const time_t&) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    if (!cfg_.simulate_game_controller) {
                        return;
                    }

                    auto gameState = std::make_unique<messages::input::gameevents::GameState>();

                    // Set up game state
                    switch(cfg_.gc_phase) {
                        case 1:
                            gameState->phase = Phase::READY;
                            break;
                        case 2:
                            gameState->phase = Phase::SET;
                            break;
                        case 3:
                            gameState->phase = Phase::PLAYING;
                            break;
                        case 4:
                            gameState->phase = Phase::TIMEOUT;
                            break;
                        case 5:
                            gameState->phase = Phase::FINISHED;
                            break;
                        case 0:
                        default:
                            gameState->phase = Phase::INITIAL;
                            break;
                    }

                    switch(cfg_.gc_mode) {
                        case 1:
                            gameState->mode = Mode::PENALTY_SHOOTOUT;
                            break;
                        case 2:
                            gameState->mode = Mode::OVERTIME;
                            break;
                        case 0:
                        default:
                            gameState->mode = Mode::NORMAL;
                            break;
                    }

                    gameState->firstHalf = cfg_.gc_first_half;
                    gameState->kickedOutByUs = cfg_.gc_kicked_out_by_us;
                    gameState->ourKickOff = cfg_.gc_our_kick_off;
                    gameState->team.teamId = cfg_.gc_team_id;
                    gameState->opponent.teamId = cfg_.gc_opponent_id;

                    // Players
                    gameState->team.players.clear();
                    gameState->team.players.push_back({0, PenaltyReason::UNPENALISED, NUClear::clock::now()});

                    switch(cfg_.gc_penalty_reason) {
                        case 1:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::BALL_MANIPULATION;
                            break;
                        case 2:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::PHYSICAL_CONTACT;
                            break;
                        case 3:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::ILLEGAL_ATTACK;
                            break;
                        case 4:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::ILLEGAL_DEFENSE;
                            break;
                        case 5:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::REQUEST_FOR_PICKUP;
                            break;
                        case 6:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::REQUEST_FOR_SERVICE;
                            break;
                        case 7:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::REQUEST_FOR_PICKUP_TO_SERVICE;
                            break;
                        case 8:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::SUBSTITUTE;
                            break;
                        case 9:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::MANUAL;
                            break;
                        case 0:
                        default:
                            gameState->team.players.at(0).penaltyReason = PenaltyReason::UNPENALISED;
                            break;
                    }

                    emit(std::move(gameState));
                });

                // Update ball position
                on<Trigger<Every<10, std::chrono::milliseconds>>>("Mock Ball Motion", [this](const time_t&) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    if (!cfg_.simulate_ball_movement) {
                        ball_velocity_ = { 0, 0 };
                        return;
                    }

                    if(cfg_.simulate_ball_velocity_decay) {
                        ball_position_[0] += ball_velocity_[0] / 100;
                        ball_position_[1] += ball_velocity_[1] / 100;

                        if((ball_velocity_[0] -= (cfg_.ball_velocity_decay / 100)) < 0) {
                            ball_velocity_[0] = 0;
                        }
                        if((ball_velocity_[1] -= (cfg_.ball_velocity_decay / 100)) < 0) {
                            ball_velocity_[1] = 0;
                        }

                    } else {
                        auto t = absolute_time();
                        double period = 40;
                        double x_amp = 3;
                        double y_amp = 2;

                        auto triangle1 = triangle_wave(t, period);
                        auto triangle2 = triangle_wave(t + (period / 4.0), period);
                        ball_position_ = { triangle1 * x_amp, triangle2 * y_amp };

                        auto velocity_x = -square_wave(t, period) * ((x_amp * 4) / period);
                        auto velocity_y = -square_wave(t + (period / 4.0), period) * ((y_amp * 4) / period);
                        ball_velocity_ = { velocity_x, velocity_y };
                    }
                });

                // // Simulate Odometry
                // on<Trigger<Every<100, std::chrono::milliseconds>>>("Mock Odometry Simulation", [this](const time_t&) {
                //     if (!cfg_.simulate_odometry) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                //         return;
                //     }

                //     auto odom = std::make_unique<messages::localisation::FakeOdometry>();

                //     double heading_diff = robot_heading_ - odom_old_robot_heading_;
                //     odom->torso_rotation = normalizeAngle(heading_diff);

                //     // Calculate torso displacement in robot-space:
                //     arma::vec2 position_diff = robot_position_ - odom_old_robot_position_;
                //     arma::mat22 rot = rotationMatrix(robot_heading_);
                //     odom->torso_displacement = rot * position_diff;

                //     odom_old_robot_position_ = robot_position_;
                //     odom_old_robot_heading_ = robot_heading_;

                //     emit(graph("Odometry torso_displacement", odom->torso_displacement[0], odom->torso_displacement[1]));
                //     emit(graph("Odometry torso_rotation", odom->torso_rotation));

                //     emit(std::move(odom));
                // });

                // Simulate Vision
                on<Trigger<Every<30, Per<std::chrono::seconds>>>, Options<Sync<MockRobot>>>("Mock Vision Simulation", [this](const time_t&) {
                    if (!cfg_.simulate_vision) {
                        return;
                    }

                    if (field_description_ == nullptr) {
                        NUClear::log(__FILE__, __LINE__, ": field_description_ == nullptr");
                        return;
                    }

                    // Sensors:
                    auto sensors = std::make_shared<messages::input::Sensors>();

                    // orientation
                    arma::vec2 robot_imu_dir_ = WorldToRobotTransform(arma::vec2({0, 0}), robot_heading_, world_imu_direction);
                    arma::mat orientation = arma::eye(3, 3);
                    orientation.submat(0, 0, 1, 0) = robot_imu_dir_;
                    orientation.submat(0, 1, 1, 1) = arma::vec2({ -robot_imu_dir_(1), robot_imu_dir_(0) });
                    sensors->orientation = orientation;

                    // orientationCamToGround
                    sensors->orientationCamToGround = arma::eye(4, 4);

                    // forwardKinematics
                    sensors->forwardKinematics[ServoID::HEAD_PITCH] = arma::eye(4, 4);

                    // Goal observation
                    if (cfg_.simulate_goal_observations) {
                        auto goals = std::make_unique<std::vector<messages::vision::Goal>>();

                        // Only observe goals that are in front of the robot
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                        arma::vec3 goal_l_pos = {0, 0, field_description_->goalpost_top_height - cfg_.camera_height};
                        arma::vec3 goal_r_pos = {0, 0, field_description_->goalpost_top_height - cfg_.camera_height};

                        if (robot_heading_ < -M_PI * 0.5 || robot_heading_ > M_PI * 0.5) {
                            goal_l_pos.rows(0, 1) = field_description_->goalpost_bl;
                            goal_r_pos.rows(0, 1) = field_description_->goalpost_br;
                        }

                        else {
                            goal_l_pos.rows(0, 1) = field_description_->goalpost_yl;
                            goal_r_pos.rows(0, 1) = field_description_->goalpost_yr;
                        }

                        if (cfg_.observe_left_goal) {
                            messages::vision::Goal goal1;
                            messages::vision::VisionObject::Measurement g1_m;
                            g1_m.position = SphericalRobotObservation(robot_position_, robot_heading_, goal_r_pos);
                            g1_m.error = arma::eye(3, 3) * 0.1;

                            // Factor in head yaw and pitch.
                            g1_m.position[0] -= headYaw;
                            g1_m.position[1] -= headPitch;

                            goal1.measurements.push_back(g1_m);
                            goal1.measurements.push_back(g1_m);

                            if (cfg_.distinguish_left_and_right_goals) {
                                goal1.side = messages::vision::Goal::Side::RIGHT;
                            }
                            
                            else {
                                goal1.side = messages::vision::Goal::Side::UNKNOWN;
                            }

                            goal1.sensors = sensors;

                            // Make sure the goal is actually within our field of view.
                            if ((std::fabs(g1_m.position[0]) < (cfg_.FOV[0] / 2)) && (std::fabs(g1_m.position[1]) < (cfg_.FOV[1] / 2))) {
                                goals->push_back(goal1);
                            }
                        }

                        if (cfg_.observe_right_goal) {
                            messages::vision::Goal goal2;
                            messages::vision::VisionObject::Measurement g2_m;
                            g2_m.position = SphericalRobotObservation(robot_position_, robot_heading_, goal_l_pos);
                            g2_m.error = arma::eye(3, 3) * 0.1;

                            // Factor in head yaw and pitch.
                            g2_m.position[0] -= headYaw;
                            g2_m.position[1] -= headPitch;

                            goal2.measurements.push_back(g2_m);
                            goal2.measurements.push_back(g2_m);

                            if (cfg_.distinguish_left_and_right_goals) {
                                goal2.side = messages::vision::Goal::Side::LEFT;
                            }
                            
                            else {
                                goal2.side = messages::vision::Goal::Side::UNKNOWN;
                            }

                            goal2.sensors = sensors;

                            // Make sure the goal is actually within our field of view.
                            if ((std::fabs(g2_m.position[0]) < (cfg_.FOV[0] / 2)) && (std::fabs(g2_m.position[1]) < (cfg_.FOV[1] / 2))) {
                                goals->push_back(goal2);
                            }
                        }

                        emit(std::move(goals));
//                        if (goals->size() > 0) {
//                            emit(std::move(goals));
//                        }
                    }

                    // Ball observation
                    if (cfg_.simulate_ball_observations) {
                        auto ball_vec = std::make_unique<std::vector<messages::vision::Ball>>();

                        messages::vision::Ball ball;
                        messages::vision::VisionObject::Measurement b_m;
                        arma::vec3 ball_pos_3d = {0, 0, field_description_->ball_radius - cfg_.camera_height};
                        ball_pos_3d.rows(0, 1) = ball_position_;
                        b_m.position = SphericalRobotObservation(robot_position_, robot_heading_, ball_pos_3d);
                        b_m.error = arma::eye(3, 3) * 0.1;

                        // Factor in head yaw and pitch.
                        b_m.position[0] -= headYaw;
                        b_m.position[1] -= headPitch;

                        ball.measurements.push_back(b_m);
                        ball.sensors = sensors;

                        // Make sure the ball is actually within our field of view.
                        if ((std::fabs(b_m.position[0]) < (cfg_.FOV[0] / 2)) && (std::fabs(b_m.position[1]) < (cfg_.FOV[1] / 2))) {
                            ball_vec->push_back(ball);
                        }

                        emit(std::move(ball_vec));
                    }

                    emit(std::make_unique<Sensors>(*sensors));
                });

                // Emit robot to NUbugger
                on<Trigger<Every<100, std::chrono::milliseconds>>, With<Mock<std::vector<messages::localisation::Self>>>, Options<Sync<MockRobot>>>("NUbugger Output", [this](const time_t&, const Mock<std::vector<messages::localisation::Self>>& mock_robots) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    auto& robots = mock_robots.data;

                    emit(graph("Actual robot position", robot_position_[0], robot_position_[1]));
                    // emit(graph("Actual robot heading", robot_heading_[0], robot_heading_[1]));
                    emit(graph("Actual robot heading", robot_heading_));
                    emit(graph("Actual robot velocity", robot_velocity_[0], robot_velocity_[1]));

                    if (robots.size() >= 1) {
                        emit(graph("Estimated robot position", robots[0].position[0], robots[0].position[1]));
                        emit(graph("Estimated robot heading", robots[0].heading[0], robots[0].heading[1]));
                    }

                    // Robot message
                    if (!cfg_.emit_robot_fieldobjects) {
                        return;
                    }

                    auto robots_msg = std::make_unique<std::vector<messages::localisation::Self>>();
                    
                    for (auto& model : robots) {
                        robots_msg->push_back(model);
                    }

                    messages::localisation::Self self_marker;
                    self_marker.position = robot_position_;
                    self_marker.heading = bearingToUnitVector(robot_heading_);
                    self_marker.sr_xx = 0.01;
                    self_marker.sr_xy = 0;
                    self_marker.sr_yy = 0.01;
                    robots_msg->push_back(self_marker);

//std::cerr << "emit(std::move(std::vector<messages::localisation::Self>));" << std::endl;
                    emit(std::move(robots_msg));
                });

                // Emit ball to Nubugger
                on<Trigger<Every<100, std::chrono::milliseconds>>, With<Mock<messages::localisation::Ball>>, With<Mock<std::vector<messages::localisation::Self>>>, Options<Sync<MockRobot>>>(
                                "NUbugger Output", [this](const time_t&, const Mock<messages::localisation::Ball>& mock_ball, const Mock<std::vector<messages::localisation::Self>>& mock_robots) {
//std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
                    auto& ball = mock_ball.data;
                    auto& robots = mock_robots.data;

                    if (robots.empty()) {
                        std::cerr << "no robots" << std::endl;
                        return;
                    }

                    arma::vec2 robot_ball_pos = RobotToWorldTransform(robots[0].position, robots[0].heading, ball.position);
                    arma::vec2 ball_pos = RobotToWorldTransform(robot_position_, robot_heading_, ball.position);
                    emit(graph("Estimated ball position", ball_pos[0], ball_pos[1]));
                    // emit(graph("Estimated ball velocity", state[2], state[3]));
                    emit(graph("Actual ball position", ball_position_[0], ball_position_[1]));
                    emit(graph("Actual ball velocity", ball_velocity_[0], ball_velocity_[1]));

                    // Ball message
                    if (!cfg_.emit_ball_fieldobjects) {
                        return;
                    }
                    
                    if (cfg_.emit_localisation_ball_vector) {
                        auto balls_msg = std::make_unique<std::vector<messages::localisation::Ball>>();


                        messages::localisation::Ball ball_model;
                        ball_model.position = ball_pos;
                        ball_model.velocity = ball_velocity_;
                        ball_model.sr_xx = ball.sr_xx;
                        ball_model.sr_xy = ball.sr_xy;
                        ball_model.sr_yy = ball.sr_yy;
                        ball_model.world_space = true;
                        balls_msg->push_back(ball_model);

                        messages::localisation::Ball ball_marker;
                        ball_marker.position = ball_position_;
                        ball_marker.velocity = ball_velocity_;
                        ball_marker.sr_xx = 0.01;
                        ball_marker.sr_xy = 0;
                        ball_marker.sr_yy = 0.01;
                        ball_marker.world_space = true;
                        balls_msg->push_back(ball_marker);
    
                        messages::localisation::Ball robot_ball;
                        robot_ball.position = robot_ball_pos;
                        robot_ball.velocity = ball_velocity_;
                        robot_ball.sr_xx = 0.01;
                        robot_ball.sr_xy = 0;
                        robot_ball.sr_yy = 0.01;
                        robot_ball.world_space = true;
                        balls_msg->push_back(robot_ball);

//                        std::cerr << "emit(std::move(std::vector<messages::localisation::Ball>));" << std::endl;
                        emit(std::move(balls_msg));
                    }

                    else {
                        auto balls_msg = std::make_unique<messages::localisation::Ball>();

                        balls_msg->position = ball_pos;
                        balls_msg->velocity = ball_velocity_;
                        balls_msg->sr_xx = 0.01;
                        balls_msg->sr_xy = 0;
                        balls_msg->sr_yy = 0.01;
                        balls_msg->world_space = true;

//std::cerr << "emit(std::move(messages::localisation::Ball));" << std::endl;
                        emit(std::move(balls_msg));
                    }
                });

                // Simulate head motion.
                on<Trigger<Every<1, std::chrono::milliseconds>>>("Mock Head Motion Simulator", [this](const time_t&) {
                        // s = s_0 + vt + a*t*t*0.5
                        // s_0 = current position
                        // t = time increment (1 ms from the trigger)
                        // v = pan speed
                        // a = 0 (head is moving with a constant velocity)
                        static size_t currentIndex = 0;

                        if ((headPans.size() > 0) && (currentIndex < headPans.size())) {
                            // Stop moving once we reach our target.
                            if (headYaw != headPans.at(currentIndex).yaw) {
                                headYaw = headYaw + (headPans.at(currentIndex).speed * 0.001);

                                // Cap movement at target position.
                                if (headYaw > headPans.at(currentIndex).yaw) {
                                    headYaw = headPans.at(currentIndex).yaw;
                                }
                            }

                            if (headPitch != headPans.at(currentIndex).pitch) {
                                headPitch = headPitch + (headPans.at(currentIndex).speed * 0.001);

                                if (headPitch > headPans.at(currentIndex).pitch) {
                                    headPitch = headPans.at(currentIndex).pitch;
                                }
                            }

                            if ((headPitch == headPans.at(currentIndex).pitch) && (headYaw == headPans.at(currentIndex).yaw)) {
                                currentIndex++;

                                if (currentIndex == headPans.size()) {
                                    currentIndex = 0;
                                    headPans.clear();
                                }
                            }
                        }
                });

                on<Trigger<LookAtAngle>>("LookAtAngle catcher", [this](const LookAtAngle& angle) {
                    if (headPans.empty()) {
                        // Select the pan speed to use.
                        double speed = (sqrt((angle.pitch * angle.pitch) + (angle.yaw * angle.yaw)) < cfg_.distance_threshold) ? cfg_.slow_speed : cfg_.fast_speed;

                        // Calculate the target yaw and pitch.
                        double yaw = std::fmin(std::fmax(angle.yaw + headYaw, cfg_.min_yaw), cfg_.max_yaw);
                        double pitch = std::fmin(std::fmax(angle.pitch + headPitch, cfg_.min_pitch), cfg_.max_pitch);

                        headPans.emplace_back(HeadPan {speed, yaw, pitch});
                    }
                });

                on<Trigger<std::vector<LookAtAngle>>>("LookAtAngles catcher", [this](const std::vector<LookAtAngle>& angles) {
                    if (headPans.empty()) {
                        double pitchLow = 0.0, pitchHigh = 0.0, yawLeft = 0.0, yawRight = 0.0;
                        double offset = cfg_.screen_padding;

                        // Loop through and get the yaw/pitch bounds.
                        for (const auto& angle : angles) {
                            pitchLow = fmin(pitchLow, angle.pitch - offset);
                            pitchHigh = fmax(pitchHigh, angle.pitch + offset);

                            yawLeft = fmin(yawLeft, angle.yaw - offset);
                            yawRight = fmax(yawRight, angle.yaw + offset);

                            offset = 0.0;
                        }

                        double avgYaw = (yawLeft + yawRight) / 2.0;
                        double avgPitch = (pitchLow + pitchHigh) / 2.0;

                        // Select the pan speed to use.
                        double speed = (sqrt((avgYaw * avgYaw) + (avgPitch * avgPitch)) < cfg_.distance_threshold) ? cfg_.slow_speed : cfg_.fast_speed;

                        // Calculate the target yaw and pitch.
                        double yaw = std::fmin(std::fmax(avgYaw + headYaw, cfg_.min_yaw), cfg_.max_yaw);
                        double pitch = std::fmin(std::fmax(avgPitch + headPitch, cfg_.min_pitch), cfg_.max_pitch);

                        headPans.emplace_back(HeadPan {speed, yaw, pitch});
                    }
                });

                on<Trigger<LookAtPosition>>("LookAtPosition catcher", [this](const LookAtPosition& position) {
                    if (headPans.empty()) {
                        // Select the pan speed to use.
                        double speed = (sqrt((position.pitch * position.pitch) + (position.yaw * position.yaw)) < cfg_.distance_threshold) ? cfg_.slow_speed : cfg_.fast_speed;

                        // Calculate the target yaw and pitch.
                        double yaw = std::fmin(std::fmax(position.yaw + headYaw, cfg_.min_yaw), cfg_.max_yaw);
                        double pitch = std::fmin(std::fmax(position.pitch + headPitch, cfg_.min_pitch), cfg_.max_pitch);

                        headPans.emplace_back(HeadPan {speed, yaw, pitch});
                    }
                });

                on<Trigger<std::vector<LookAtPosition>>>("LookAtPositions catcher", [this](const std::vector<LookAtPosition>& positions) {
                    if (headPans.empty()) {
                        double currentYaw = headYaw;
                        double currentPitch = headPitch;
                        std::vector<LookAtPosition> nonConstPositions;

                        // Make a sortable vector.
                        for (auto& position : positions) {
                            nonConstPositions.emplace_back(position);
                        }

                        // Sort the positions into order of closest to current yaw/pitch.
                        std::stable_sort(nonConstPositions.begin(), nonConstPositions.end(), [&currentYaw, &currentPitch] (const LookAtPosition& a, const LookAtPosition& b) {
                            const double diffx_a = currentYaw - a.yaw;
                            const double diffx_b = currentYaw - b.yaw;
                            const double diffy_a = currentPitch - a.pitch;
                            const double diffy_b = currentPitch - b.pitch;
                            const double dist_a = (diffx_a * diffx_a) + (diffy_a * diffy_a);
                            const double dist_b = (diffx_b * diffx_b) + (diffy_b * diffy_b);

                            return (dist_a < dist_b);
                        });

                        // Do the pan.
                        double speed = cfg_.slow_speed;

                        for (auto& position : nonConstPositions) {
                            headPans.emplace_back(HeadPan {speed, position.yaw, position.pitch});
                            speed = cfg_.fast_speed;
                        }
                    }
                });
                    
                // Give the ball velocity when it is kicked
                on<Trigger<KickCommand>>([this] (const KickCommand& kickCommand) {
                    auto direction = kickCommand.direction;
                    auto leg = kickCommand.leg;

                    int quadrant = getDirectionalQuadrant(direction[0], direction[1]);

                    // check if the command was valid
                    bool valid = true;
                    if (leg == LimbID::RIGHT_LEG) {
                        if (quadrant == 2 || quadrant == 3) {
                            NUClear::log<NUClear::WARN>("Right leg cannot kick towards: ", direction);
                            valid = false;
                        }
                    } else if (leg == LimbID::LEFT_LEG) {
                        if (quadrant == 2 || quadrant == 1) {
                            NUClear::log<NUClear::WARN>("Left leg cannot kick towards: ", direction);
                            valid = false;
                        }
                    } else {
                        NUClear::log<NUClear::WARN>("Cannot kick with limb: ", uint(leg));
                        valid = false;
                    }

                    if (valid) {
                        arma::vec2 kick_direction = arma::normalise(RobotToWorldTransform(robot_position_, robot_heading_, direction));

                        ball_velocity_ = {cfg_.initial_kick_velocity * kick_direction[0], cfg_.initial_kick_velocity * kick_direction[1]};
                    }

                    emit(std::move(std::make_unique<KickFinished>()));
                });
            }
        }
    }
}
