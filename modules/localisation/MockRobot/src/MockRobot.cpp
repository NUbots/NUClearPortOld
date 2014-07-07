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
// #include "BallModel.h"

using utility::nubugger::graph;
using messages::support::Configuration;
using messages::support::FieldDescription;
using modules::localisation::MockRobotConfig;
using messages::localisation::Mock;

namespace modules {
namespace localisation {

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

    void MockRobot::UpdateConfiguration(
        const messages::support::Configuration<MockRobotConfig>& config) {
        cfg_.simulate_vision = config["SimulateVision"].as<bool>();
        cfg_.simulate_goal_observations = config["SimulateGoalObservations"].as<bool>();
        cfg_.simulate_ball_observations = config["SimulateBallObservations"].as<bool>();
        cfg_.simulate_odometry = config["SimulateOdometry"].as<bool>();
        cfg_.simulate_robot_movement = config["SimulateRobotMovement"].as<bool>();
        cfg_.simulate_ball_movement = config["SimulateBallMovement"].as<bool>();
        cfg_.emit_robot_fieldobjects = config["EmitRobotFieldobjects"].as<bool>();
        cfg_.emit_ball_fieldobjects = config["EmitBallFieldobjects"].as<bool>();

        NUClear::log(__func__, "cfg_.simulate_vision = ", cfg_.simulate_vision);
        NUClear::log(__func__, "cfg_.simulate_goal_observations = ", cfg_.simulate_goal_observations);
        NUClear::log(__func__, "cfg_.simulate_ball_observations = ", cfg_.simulate_ball_observations);
        NUClear::log(__func__, "cfg_.simulate_odometry = ", cfg_.simulate_odometry);
        NUClear::log(__func__, "cfg_.simulate_robot_movement = ", cfg_.simulate_robot_movement);
        NUClear::log(__func__, "cfg_.simulate_ball_movement = ", cfg_.simulate_ball_movement);
        NUClear::log(__func__, "cfg_.emit_robot_fieldobjects = ", cfg_.emit_robot_fieldobjects);
        NUClear::log(__func__, "cfg_.emit_ball_fieldobjects = ", cfg_.emit_ball_fieldobjects);
    }

    MockRobot::MockRobot(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<FieldDescription>>("FieldDescription Update", [this](const FieldDescription& desc) {
            field_description_ = std::make_shared<FieldDescription>(desc);
        });

        on<Trigger<Configuration<MockRobotConfig>>>(
            "MockRobotConfig Update",
            [this](const Configuration<MockRobotConfig>& config) {
            UpdateConfiguration(config);
        });

        // Update robot position
        on<Trigger<Every<10, std::chrono::milliseconds>>>("Robot motion", [this](const time_t&){
            if (!cfg_.simulate_robot_movement) {
                robot_velocity_ = { 0, 0 };
                return;
            }

            auto t = absolute_time();
            double period = 100;
            double x_amp = 3;
            double y_amp = 2;

            arma::vec old_pos = robot_position_;

            auto wave1 = triangle_wave(t, period);
            auto wave2 = triangle_wave(t + (period / 4.0), period);
            // auto wave1 = sine_wave(t, period);
            // auto wave2 = sine_wave(t + (period / 4.0), period);
            robot_position_ = { wave1 * x_amp, wave2 * y_amp };

            arma::vec diff = robot_position_ - old_pos;

            robot_heading_ = arma::normalise(diff);
            robot_velocity_ = robot_heading_ / 100.0;
        });

        // Update ball position
        on<Trigger<Every<10, std::chrono::milliseconds>>>("Ball motion", [this](const time_t&){

            if (!cfg_.simulate_ball_movement) {
                ball_velocity_ = { 0, 0 };
                return;
            }


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
        });

//         // Simulate orientation matrix
//         on<Trigger<Every<10, std::chrono::milliseconds>>>(
//             "Orientation Matrix Simulation", [this](const time_t&){
// // orient =   M: W -> R
// //          M^T: R -> W

// //          a, x_w,  M*R_a*x_w = x_r

// // M is an orthonormal basis for world coords expressed in robot coords
// // i.e. M contains unit vectors pointing along each of the world axes
// // Note: M can only attempt to track the robot's orientation - not its position.
// //       i.e. The origin of the world coords resulting from M is still the
// //            robot's torso, but the axes are parallel to the field axes.
//             arma::mat33 M =


//         });

        // Simulate Odometry
        on<Trigger<Every<100, std::chrono::milliseconds>>>("Odometry Simulation",
            [this](const time_t&) {
            if (!cfg_.simulate_odometry)
                return;

            auto odom = std::make_unique<messages::localisation::FakeOdometry>();

            double old_heading = std::atan2(odom_old_robot_heading_[1],
                                            odom_old_robot_heading_[0]);
            double new_heading = std::atan2(robot_heading_[1],
                                            robot_heading_[0]);
            double heading_diff = new_heading - old_heading;
            odom->torso_rotation = utility::math::angle::normalizeAngle(heading_diff);

            // Calculate torso displacement in robot-space:
            arma::vec2 position_diff = robot_position_ - odom_old_robot_position_;
            double theta = -new_heading;
            arma::mat22 rot;
            //NOTE: Matrix initialisation changed by jake fountain 29th may 2014
            rot <<  std::cos(theta) << std::sin(theta) << arma::endr
                << -std::sin(theta) << std::cos(theta);
            // Rotate position_diff by -new_heading.
            odom->torso_displacement = rot * position_diff;


            odom_old_robot_position_ = robot_position_;
            odom_old_robot_heading_ = robot_heading_;

            emit(graph("Odometry torso_displacement",
                odom->torso_displacement[0],
                odom->torso_displacement[1]));
            emit(graph("Odometry torso_rotation", odom->torso_rotation));

            emit(std::move(odom));
        });

        // Simulate Vision
        on<Trigger<Every<200, std::chrono::milliseconds>>,
           Options<Sync<MockRobot>>>("Vision Simulation", [this](const time_t&) {
            if (!cfg_.simulate_vision)
                return;

            // Camera setup
            auto camera_pos = arma::vec3 { robot_position_[0], robot_position_[1], 0.0 };
            double camera_heading = std::atan2(robot_heading_[1], robot_heading_[0]);

            // Goal observation
            if (cfg_.simulate_goal_observations && field_description_ != nullptr) {
                auto& fd = field_description_;
                auto goal1_pos = arma::vec3 { fd->goalpost_br[0], fd->goalpost_br[1], 0.0 };
                auto goal2_pos = arma::vec3 { fd->goalpost_bl[0], fd->goalpost_bl[1], 0.0 };

                auto goal1 = messages::vision::Goal();
                auto goal2 = messages::vision::Goal();
                goal1.side = messages::vision::Goal::Side::RIGHT;
                goal2.side = messages::vision::Goal::Side::LEFT;

                // // Observations in spherical from camera: (dist, bearing, declination)
                // messages::vision::VisionObject::Measurement g1_m;
                // messages::vision::VisionObject::Measurement g2_m;
                // g1_m.sphericalFromCamera = utility::math::coordinates::Cartesian2Spherical(goal1_pos - camera_pos);
                // g2_m.sphericalFromCamera = utility::math::coordinates::Cartesian2Spherical(goal2_pos - camera_pos);
                // g1_m.sphericalFromCamera[1] = utility::math::angle::normalizeAngle(goal1.sphericalFromCamera[1] - camera_heading);
                // g2_m.sphericalFromCamera[1] = utility::math::angle::normalizeAngle(goal2.sphericalFromCamera[1] - camera_heading);
                // g1_m.error = arma::eye(3, 3) * 0.1;
                // g2_m.error = arma::eye(3, 3) * 0.1;
                // goal1.measurements.push_back(g1_m);
                // goal2.measurements.push_back(g2_m);

                // Observations in robot-relative cartesian:
                messages::vision::VisionObject::Measurement g1_m;
                messages::vision::VisionObject::Measurement g2_m;
                auto rot = utility::math::matrix::zRotationMatrix(camera_heading);
                g1_m.position = rot * (goal1_pos - camera_pos);
                g2_m.position = rot * (goal2_pos - camera_pos);
                g1_m.error = arma::eye(3, 3) * 0.1;
                g2_m.error = arma::eye(3, 3) * 0.1;
                goal1.measurements.push_back(g1_m);
                goal2.measurements.push_back(g2_m);

                auto goals = std::make_unique<std::vector<messages::vision::Goal>>();

                goals->push_back(goal1);
                goals->push_back(goal2);

                emit(std::move(goals));
            }

            // Ball observation
            if (cfg_.simulate_ball_observations) {
                auto ball_vec = std::make_unique<std::vector<messages::vision::Ball>>();

                messages::vision::Ball ball;
                messages::vision::VisionObject::Measurement b_m;
                auto ball_pos = arma::vec3 { ball_position_[0], ball_position_[1], 0.0 };
                
                // // Observations in spherical from camera: (dist, bearing, declination)
                // b_m.sphericalFromCamera = utility::math::coordinates::Cartesian2Spherical(ball_pos - camera_pos);
                // b_m.sphericalFromCamera[1] = utility::math::angle::normalizeAngle(ball.sphericalFromCamera[1] - camera_heading);
                // b_m.error = arma::eye(3, 3) * 0.1;

                // Observations in robot-relative cartesian:
                auto rot = utility::math::matrix::zRotationMatrix(camera_heading);
                b_m.position = rot * (ball_pos - camera_pos);
                b_m.error = arma::eye(3, 3) * 0.1;

                ball.measurements.push_back(b_m);
                ball_vec->push_back(ball);

                emit(std::move(ball_vec));
            }
        });

        // Emit robot to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           With<std::vector<messages::localisation::Self>>,
           Options<Sync<MockRobot>>>("NUbugger Output",
            [this](const time_t&,
                   const std::vector<messages::localisation::Self>& robots) {

            emit(graph("Actual robot position", robot_position_[0], robot_position_[1]));
            emit(graph("Actual robot heading", robot_heading_[0], robot_heading_[1]));
            emit(graph("Actual robot velocity", robot_velocity_[0], robot_velocity_[1]));

            if (robots.size() >= 1) {
                emit(graph("Estimated robot position", robots[0].position[0], robots[0].position[1]));
                emit(graph("Estimated robot heading", robots[0].heading[0], robots[0].heading[1]));
            }

            // Robot message
            if (!cfg_.emit_robot_fieldobjects)
                return;

            auto robot_msg = std::make_unique<messages::localisation::FieldObject>();
            std::vector<messages::localisation::FieldObject::Model> robot_msg_models;

            // for (auto& model : robots) {
            //     messages::localisation::FieldObject::Model robot_model;
            //     robot_msg->name = "self";
            //     robot_model.wm_x = model.position[0];
            //     robot_model.wm_y = model.position[1];
            //     robot_model.heading = std::atan2(model.heading[1], model.heading[0]);
            //     robot_model.sd_x = 1;
            //     robot_model.sd_y = 0.25;
            //     robot_model.sr_xx = model.sr_xx; // * 100;
            //     robot_model.sr_xy = model.sr_xy; // * 100;
            //     robot_model.sr_yy = model.sr_yy; // * 100;
            //     robot_model.lost = false;
            //     robot_msg_models.push_back(robot_model);
            // }

            messages::localisation::FieldObject::Model actual_robot;
            robot_msg->name = "self";
            actual_robot.wm_x = robot_position_[0];
            actual_robot.wm_y = robot_position_[1];
            actual_robot.heading = std::atan2(robot_heading_[1], robot_heading_[0]);
            actual_robot.sd_x = 1;
            actual_robot.sd_y = 0.25;
            actual_robot.sr_xx = 0.01;
            actual_robot.sr_xy = 0.0;
            actual_robot.sr_yy = 0.01;
            actual_robot.lost = false;
            robot_msg_models.push_back(actual_robot);

            robot_msg->models = robot_msg_models;
            emit(std::move(robot_msg));
        });

        // Emit ball to Nubugger
        on<Trigger<Every<100, std::chrono::milliseconds>>,
           With<Mock<messages::localisation::Ball>>,
           With<Mock<std::vector<messages::localisation::Self>>>,
           Options<Sync<MockRobot>>>("NUbugger Output",
            [this](const time_t&,
                   const Mock<messages::localisation::Ball>& mock_ball,
                   const Mock<std::vector<messages::localisation::Self>>& mock_robots) {

            auto& ball = mock_ball.data;
            auto& robots = mock_robots.data;

            arma::vec2 ball_pos = utility::localisation::transform::RobotBall2FieldBall(
                robot_position_, robot_heading_, ball.position);

            if (robots.empty())
                return;

            arma::vec2 robot_ball_pos = utility::localisation::transform::RobotBall2FieldBall(
                robots[0].position, robots[0].heading, ball.position);

            emit(graph("Estimated ball position", ball_pos[0], ball_pos[1]));
            // emit(graph("Estimated ball velocity", state[2], state[3]));
            emit(graph("Actual ball position", ball_position_[0], ball_position_[1]));
            emit(graph("Actual ball velocity", ball_velocity_[0], ball_velocity_[1]));

            // Ball message
            if (!cfg_.emit_ball_fieldobjects)
                return;
            
            auto ball_msg = std::make_unique<messages::localisation::FieldObject>();
            std::vector<messages::localisation::FieldObject::Model> ball_msg_models;
            // messages::localisation::FieldObject::Model ball_model;
            // ball_msg->name = "ball";
            // ball_model.wm_x = ball_pos[0];
            // ball_model.wm_y = ball_pos[1];
            // ball_model.heading = 0;
            // ball_model.sd_x = 0.1;
            // ball_model.sd_y = 0.1;
            // ball_model.sr_xx = ball.sr_xx;
            // ball_model.sr_xy = ball.sr_xy;
            // ball_model.sr_yy = ball.sr_yy;
            // ball_model.lost = false;
            // ball_msg_models.push_back(ball_model);

            messages::localisation::FieldObject::Model ball_marker_model;
            ball_msg->name = "ball";
            ball_marker_model.wm_x = ball_position_[0];
            ball_marker_model.wm_y = ball_position_[1];
            ball_marker_model.heading = 0;
            ball_marker_model.sd_x = 0.01;
            ball_marker_model.sd_y = 0.01;
            ball_marker_model.sr_xx = 0.01;
            ball_marker_model.sr_xy = 0;
            ball_marker_model.sr_yy = 0.01;
            ball_marker_model.lost = false;
            ball_msg_models.push_back(ball_marker_model);

            // messages::localisation::FieldObject::Model robot_ball_model;
            // ball_msg->name = "ball";
            // robot_ball_model.wm_x = robot_ball_pos[0];
            // robot_ball_model.wm_y = robot_ball_pos[1];
            // robot_ball_model.heading = 0;
            // robot_ball_model.sd_x = 0.005;
            // robot_ball_model.sd_y = 0.005;
            // robot_ball_model.sr_xx = 0.01;
            // robot_ball_model.sr_xy = 0;
            // robot_ball_model.sr_yy = 0.01;
            // robot_ball_model.lost = false;
            // ball_msg_models.push_back(robot_ball_model);

            ball_msg->models = ball_msg_models;
            emit(std::move(ball_msg));
        });
    }
}
}

