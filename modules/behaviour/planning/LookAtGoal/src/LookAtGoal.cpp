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

#include "LookAtGoal.h"

#include "messages/localisation/FieldObject.h"
#include "messages/behaviour/LookStrategy.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"

#include "utility/motion/InverseKinematics.h"

namespace modules {
    namespace behaviour {
        namespace planning {

		using messages::vision::Ball;
		using messages::vision::Goal;
		using messages::behaviour::LookAtPoint;
		using messages::behaviour::LookAtPosition;
		using messages::behaviour::HeadBehaviourConfig;
		using messages::input::Sensors;
		using messages::support::Configuration;
		using messages::behaviour::LookAtPosition;
		using messages::behaviour::LookAtGoalStart;
		using messages::behaviour::LookAtGoalStop;

		LookAtGoal::LookAtGoal(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

			on<Trigger<Configuration<HeadBehaviourConfig>>>([this](const Configuration<HeadBehaviourConfig>& config) {
				BALL_SEARCH_TIMEOUT_MILLISECONDS = config["BALL_SEARCH_TIMEOUT_MILLISECONDS"].as<float>();
				X_FACTOR = config["X_FACTOR"].as<float>();	
				Y_FACTOR = config["Y_FACTOR"].as<float>();	
				BALL_UNCERNTAINTY_THRESHOLD = config["BALL_UNCERTAINTY_THRESHOLD"].as<float>();	
			});

               on<Trigger<Startup>>([this](const Startup&) {
					handle.disable();
                });


			on<Trigger<LookAtGoalStart>>([this](const LookAtGoalStart&) {
std::cerr << "LookAtGoal - enabled" << std::endl;
				handle.enable();
			});

			on<Trigger<LookAtGoalStop>>([this](const LookAtGoalStop&) {
std::cerr << "LookAtGoal - disabled" << std::endl;
				handle.disable();
			});

			handle = on<Trigger<std::vector<Goal>>, With<std::vector<Ball>>, With<Sensors>, With<Optional<messages::localisation::Ball>>>
				([this] (const std::vector<Goal>& goals, 
					const std::vector<Ball>& balls, 
					const Sensors& sensors, 
					const std::shared_ptr<const messages::localisation::Ball>& ball
				) {
std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
				if (goals.size() > 0) {
					timeSinceLastSeen = sensors.timestamp;
					std::vector<LookAtPosition> angles;
					angles.reserve(10);
					std::vector<Goal> nonConstGoals;
					arma::vec2 screenAngular;

					nonConstGoals.reserve(goals.size());

					// Copy the const goals into the non-const vector.
					for (auto& g : goals) {
						nonConstGoals.emplace_back(g);
					}

					// Sort the goals into order of closest to centre-screen.
					std::stable_sort(nonConstGoals.begin(), nonConstGoals.end(), [] (const Goal& a, const Goal& b) {
						return (arma::norm(a.screenAngular, 2) < arma::norm(b.screenAngular, 2));
					});

					// Add all goals to angles in order of which goal is closest to centre-screen.
					for (auto& g : goals) {
						angles.emplace_back(LookAtPosition {g.screenAngular[0], -g.screenAngular[1]});
					}

					if (balls.size() > 0) {
						angles.emplace_back(LookAtPosition {balls[0].screenAngular[0], -balls[0].screenAngular[1]});
					}

					else if ((ball != NULL) && ((ball->sr_xx > BALL_UNCERNTAINTY_THRESHOLD) || (ball->sr_yy > BALL_UNCERNTAINTY_THRESHOLD))) {
					double xFactor = X_FACTOR * std::sqrt(ball->sr_xx);
					double yFactor = Y_FACTOR * std::sqrt(ball->sr_yy);
		
					screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0], ball->position[1], 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
					angles.emplace_back(LookAtPosition {screenAngular[0], screenAngular[1]});

					screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0] + xFactor, ball->position[1], 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
					angles.emplace_back(LookAtPosition {screenAngular[0], screenAngular[1]});

					screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0] - xFactor, ball->position[1], 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
					angles.emplace_back(LookAtPosition {screenAngular[0], screenAngular[1]});

					screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0], ball->position[1] + yFactor, 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
					angles.emplace_back(LookAtPosition {screenAngular[0], screenAngular[1]});

					screenAngular = utility::motion::kinematics::calculateHeadJointsToLookAt({ball->position[0], ball->position[1] - yFactor, 0}, sensors.orientationCamToGround, sensors.orientationBodyToGround);
					angles.emplace_back(LookAtPosition {screenAngular[0], screenAngular[1]});
				} 
				
				emit(std::make_unique<std::vector<LookAtPosition>>(angles));
			} 

			else if(std::chrono::duration<float, std::ratio<1,1000>>(sensors.timestamp - timeSinceLastSeen).count() > BALL_SEARCH_TIMEOUT_MILLISECONDS) {
					//do a blind scan'n'pan
					//XXX: this needs to be a look at sequence rather than a look at point
					std::vector<LookAtPosition> angles;
std::cerr << __FILE__ << ", " << __LINE__ << ": " << __func__ << std::endl;
				// angles.emplace_back(LookAtPosition {-M_PI / 2, -M_PI / 4});
				// angles.emplace_back(LookAtPosition {0, -M_PI / 4});
				// angles.emplace_back(LookAtPosition {M_PI / 2, -M_PI / 4});
				// angles.emplace_back(LookAtPosition {M_PI / 2, 0});
				// angles.emplace_back(LookAtPosition {0, 0});
				// angles.emplace_back(LookAtPosition {-M_PI / 2, 0});
				// angles.emplace_back(LookAtPosition {-M_PI / 2, M_PI / 4});
				// angles.emplace_back(LookAtPosition {0, M_PI / 4});
				// angles.emplace_back(LookAtPosition {M_PI / 2, M_PI / 4});

				// angles.emplace_back(LookAtPosition {M_PI / 2, -M_PI / 4});
				// angles.emplace_back(LookAtPosition {0, -M_PI / 4});
				// angles.emplace_back(LookAtPosition {-M_PI / 2, -M_PI / 4});
				// angles.emplace_back(LookAtPosition {-M_PI / 2, 0});
				// angles.emplace_back(LookAtPosition {0, 0});
				// angles.emplace_back(LookAtPosition {M_PI / 2, 0});
				// angles.emplace_back(LookAtPosition {M_PI / 2, M_PI / 4});
				// angles.emplace_back(LookAtPosition {0, M_PI / 4});
				// angles.emplace_back(LookAtPosition {-M_PI / 2, M_PI / 4});
					const double scanYaw = 1.5;
				const double scanPitch = 0.8;

				const size_t panPoints = 4;

				for (size_t i = 0; i < panPoints+1; ++i) {
					angles.emplace_back(LookAtPosition {i * scanYaw / panPoints - scanYaw / 2.0, -scanPitch * (i % 2) + scanPitch});
				}

				for (size_t i = 0; i < panPoints+1; ++i) {
					angles.emplace_back(LookAtPosition {-(i * scanYaw / panPoints - scanYaw / 2.0), -scanPitch * ((i + 1) % 2) + scanPitch});
				}

				emit(std::make_unique<std::vector<LookAtPosition>>(angles));
				}
			});
		}
        }  // planning
    }  // behaviours
}  // modules
