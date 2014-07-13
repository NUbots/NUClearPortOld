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

#include "SoccerStrategy.h"

#include "messages/behaviour/LookStrategy.h"
#include "messages/behaviour/WalkStrategy.h"
#include "messages/behaviour/KickPlan.h"
#include "messages/input/Sensors.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/support/Configuration.h"
#include "messages/motion/GetupCommand.h"
#include "messages/motion/DiveCommand.h"
#include "messages/output/Say.h"

#include "utility/math/angle.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/ParametricLine.h"
#include "utility/localisation/transform.h"
#include "utility/support/armayamlconversions.h"

namespace modules {
    namespace behaviour {
        namespace strategy {

		using messages::localisation::Ball;
		using messages::localisation::Self;
		using messages::behaviour::LookAtBallStart;
		using messages::behaviour::LookAtBallStop;
		using messages::behaviour::LookAtGoalStart;
		using messages::behaviour::LookAtGoalStop;
		using messages::behaviour::LookAtAngle;
		using messages::behaviour::LookAtPoint;
		using messages::behaviour::LookAtPosition;
		using messages::behaviour::HeadBehaviourConfig;
		using messages::input::Sensors;
		using messages::platform::darwin::DarwinSensors;
		using messages::platform::darwin::ButtonLeftDown;
		using messages::platform::darwin::ButtonMiddleDown;
		using messages::support::Configuration;
		using messages::support::FieldDescription;
		using messages::motion::KickCommand;
		using messages::motion::WalkCommand;
		using messages::behaviour::WalkStrategy;
		using messages::behaviour::WalkTarget;
		using messages::behaviour::WalkApproach;
		using messages::behaviour::KickPlan;

		using messages::input::gameevents::GameState;
		using messages::input::gameevents::Phase;
		using messages::input::gameevents::Mode;
		using messages::input::gameevents::PenaltyReason;

		using messages::output::Say;

		using utility::math::geometry::Polygon;
		using utility::math::geometry::Plane;
		using utility::math::geometry::ParametricLine;

		SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
			on<Trigger<Configuration<SoccerStrategyConfig>>>([this](const Configuration<SoccerStrategyConfig>& config) {
				Zone zone;

				MAX_BALL_DISTANCE = config["MAX_BALL_DISTANCE"].as<float>();
				KICK_DISTANCE_THRESHOLD = config["KICK_DISTANCE_THRESHOLD"].as<float>();
				BALL_CERTAINTY_THRESHOLD = config["BALL_CERTAINTY_THRESHOLD"].as<float>();
				BALL_SELF_INTERSECTION_REGION = config["BALL_SELF_INTERSECTION_REGION"].as<float>();
				BALL_LOOK_ERROR = config["BALL_LOOK_ERROR"].as<arma::vec2>();
				BALL_MOVEMENT_THRESHOLD = config["BALL_MOVEMENT_THRESHOLD"].as<float>();
				GOAL_LOOK_ERROR = config["GOAL_LOOK_ERROR"].as<arma::vec2>();
				ANGLE_THRESHOLD = config["ANGLE_THRESHOLD"].as<float>();
				POSITION_THRESHOLD_TIGHT = config["POSITION_THRESHOLD_TIGHT"].as<float>();
				POSITION_THRESHOLD_LOOSE = config["POSITION_THRESHOLD_LOOSE"].as<float>();
				IS_GOALIE = config["GOALIE"].as<bool>();
				MY_ZONE = config["MY_ZONE"].as<int>();

				ZONES.reserve(4);

				try {
					zone.defaultPosition = config["ZONE_0_DEFAULT_POSITION"].as<arma::vec2>();
					zone.startPosition = config["ZONE_0_START_POSITION"].as<arma::vec2>();
					zone.zone.set(config["ZONE_0"].as<std::vector<arma::vec2>>());
					ZONES.emplace_back(zone);

					zone.defaultPosition = config["ZONE_1_DEFAULT_POSITION"].as<arma::vec2>();
					zone.startPosition = config["ZONE_1_START_POSITION"].as<arma::vec2>();
					zone.zone.set(config["ZONE_1"].as<std::vector<arma::vec2>>());
					ZONES.emplace_back(zone);

					zone.defaultPosition = config["ZONE_2_DEFAULT_POSITION"].as<arma::vec2>();
					zone.startPosition = config["ZONE_2_START_POSITION"].as<arma::vec2>();
					zone.zone.set(config["ZONE_2"].as<std::vector<arma::vec2>>());
					ZONES.emplace_back(zone);

					zone.defaultPosition = config["ZONE_3_DEFAULT_POSITION"].as<arma::vec2>();
					zone.startPosition = config["ZONE_3_START_POSITION"].as<arma::vec2>();
					zone.zone.set(config["ZONE_3"].as<std::vector<arma::vec2>>());
					ZONES.emplace_back(zone);
				}

				catch (const std::domain_error& e) {
					throw std::domain_error("SoccerStrategy::on<Trigger<Configuration<SoccerStrategyConfig>>> - Invalid zone description!");
				}
			});

			on<Trigger<Startup>>([this](const Startup&) {
				penalisedButtonStatus = false;
				feetOffGround = true;
				isKicking = false;
				isWalking = false;
				lookingAtBall = false;
				lookingAtGoal = false;

				currentState.primaryGameState = GameStatePrimary::INITIAL;
				currentState.secondaryGameState = GameStateSecondary::NORMAL;

				stopMoving();
			});

			on<Trigger<ButtonLeftDown>>([this](const ButtonLeftDown&) {
				currentState.primaryGameState++;

				switch (currentState.primaryGameState) {
					case GameStatePrimary::INITIAL: {
						emit(std::move(std::make_unique<messages::output::Say>("Initial")));
						std::cerr << "initial" << std::endl;
						break;
					}

					case GameStatePrimary::READY: {
						emit(std::move(std::make_unique<messages::output::Say>("Ready")));
						std::cerr << "ready" << std::endl;
						break;
					}

					case GameStatePrimary::SET: {
						emit(std::move(std::make_unique<messages::output::Say>("Set")));
						std::cerr << "set" << std::endl;
						break;
					}

					case GameStatePrimary::PLAYING: {
						emit(std::move(std::make_unique<messages::output::Say>("Playing")));
						std::cerr << "playing" << std::endl;
						break;
					}

					case GameStatePrimary::FINISHED: {
						emit(std::move(std::make_unique<messages::output::Say>("Finished")));
						std::cerr << "finished" << std::endl;
						break;
					}

					default: {
						emit(std::move(std::make_unique<messages::output::Say>("Undefined State. Something broke")));
						std::cerr << "Undefined State. Something broke" << std::endl;
						break;
					}
				}
			});

			on<Trigger<ButtonMiddleDown>>([this](const ButtonMiddleDown&) {
				// Am I penalised?
				if (!currentState.penalised) {
					currentState.penalised = true;
					emit(std::move(std::make_unique<messages::output::Say>("Penalised")));
				}

				else {
					currentState.penalised = false;
					emit(std::move(std::make_unique<messages::output::Say>("Unpenalised")));
				}
			});

			// Check to see if both feet are on the ground.
			on<Trigger<messages::input::Sensors>>([this](const messages::input::Sensors& sensors) {
				feetOffGround = (!sensors.leftFootDown && !sensors.rightFootDown);
			});

			// Check to see if we are currently in the process of getting up.
			on<Trigger<messages::motion::ExecuteGetup>>([this](const messages::motion::ExecuteGetup&) {
				isGettingUp = true;
			});

			// Check to see if we have finished getting up.
			on<Trigger<messages::motion::KillGetup>>([this](const messages::motion::KillGetup&) {
				isGettingUp = false;
			});

			// Check to see if we are currently in the process of diving.
			on<Trigger<messages::motion::DiveCommand>>([this](const messages::motion::DiveCommand&) {
				isDiving = true;
			});

			// Check to see if we have finished diving.
			on<Trigger<messages::motion::DiveFinished>>([this](const messages::motion::DiveFinished&) {
				isDiving = false;
			});

			// Check to see if we are about to kick.
			on<Trigger<messages::motion::KickCommand>>([this](const messages::motion::KickCommand&) {
				isKicking = true;
			});

			// Check to see if the kick has finished.
			on<Trigger<messages::motion::KickFinished>>([this](const messages::motion::KickFinished&) {
				isKicking = false;
			});

			// Check to see if we are walking.
			on<Trigger<messages::motion::WalkStartCommand>, With<messages::motion::WalkCommand>>
				([this](const messages::motion::WalkStartCommand&,
					const messages::motion::WalkCommand&) {
				isWalking = true;
			});

			// Check to see if we are no longer walking.
			on<Trigger<messages::motion::WalkStopped>>([this](const messages::motion::WalkStopped&) {
				isWalking = false;
			});

			// Get the field description.
			on<Trigger<Startup>,
	           With<Optional<FieldDescription>>>("FieldDescription Update",
	           [this](const Startup&, const std::shared_ptr<const FieldDescription>& desc) {
	            if (desc == nullptr) {
	                NUClear::log(__FILE__, ", ", __LINE__, ": FieldDescription Update: SoccerConfig module might not be installed.");
	                throw std::runtime_error("FieldDescription Update: SoccerConfig module might not be installed");
	            }

	            FIELD_DESCRIPTION = *desc;
	        });

			// Check to see if we are looking at the ball.
			on<Trigger<LookAtBallStart>>([this](const LookAtBallStart&) {
				lookingAtBall = true;
			});

			// Check to see if we are no longer looking at the ball.
			on<Trigger<LookAtBallStop>>([this](const LookAtBallStop&) {
				lookingAtBall = false;
			});

			// Check to see if we are looking at the goals.
			on<Trigger<LookAtGoalStart>>([this](const LookAtGoalStart&) {
				lookingAtGoal = true;
			});

			// Check to see if we are no longer looking at the goals.
			on<Trigger<LookAtGoalStop>>([this](const LookAtGoalStop&) {
				lookingAtGoal = false;
			});

			//Main Loop?
			on<Trigger<Every<30, Per<std::chrono::seconds>>>,
				With<messages::localisation::Ball>,
				With<std::vector<messages::localisation::Self>>,
				With<messages::input::gameevents::GameState>,
				Options<Single>>([this](const time_t&,
							const messages::localisation::Ball& ball,
							const std::vector<messages::localisation::Self>& selfs,
							const messages::input::gameevents::GameState& gameState
							) {

					// Calculate the position of the ball in field coordinates.
					arma::vec2 globalBallPosition = utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.position);

					// Make a copy of the previous state.
					memcpy(&previousState, &currentState, sizeof(State));

					// Make a copy of the ball.
					memcpy(&currentState.ball, &ball, sizeof(Ball));

					// Store current position and heading.
					if (selfs.size() > 0) {
						currentState.position = selfs.at(0).position;
						currentState.heading = selfs.at(0).heading;
					}
					
					else {
						std::cerr << "SoccerStrategy - No Self!!!!" << std::endl;
					}


					// Parse game controller state infoirmation as well as button pushes.
					updateGameState(gameState);

					// Are we kicking off?
					currentState.kickOff = gameState.ourKickOff;

					// Am I the kicker?
					// Is my start position inside the centre circle?
					currentState.kicker = ((arma::norm(ZONES.at(MY_ZONE).startPosition, 2) < (FIELD_DESCRIPTION.dimensions.center_circle_diameter / 2)) && (currentState.primaryGameState == GameStatePrimary::READY ||
								currentState.primaryGameState == GameStatePrimary::SET || currentState.primaryGameState == GameStatePrimary::PLAYING));

//					currentState.kicker = ((arma::norm(ZONES.at(MY_ZONE).startPosition, 2) < (FIELD_DESCRIPTION.dimensions.center_circle_diameter / 2)) && (currentState.primaryGameState == GameStatePrimary::READY ||
//								currentState.primaryGameState == GameStatePrimary::SET || currentState.primaryGameState == GameStatePrimary::PLAYING)) || ((currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK ||
//								currentState.secondaryGameState == GameStateSecondary::FREE_KICK || currentState.secondaryGameState == GameStateSecondary::GOAL_KICK || currentState.secondaryGameState == GameStateSecondary::CORNER_KICK) &&
//								currentState.primaryGameState == GameStatePrimary::PLAYING); // && currentState.???);

					// Have I been picked up?
					currentState.pickedUp = feetOffGround && !isGettingUp && !isDiving;

					// Have I been penalised or unpenalised?
					if (gameState.team.players.at(0).penaltyReason != PenaltyReason::UNPENALISED && !previousState.penalised) {
						currentState.penalised = true;
						emit(std::move(std::make_unique<messages::output::Say>("Penalised")));
						
					}

					else if (gameState.team.players.at(0).penaltyReason == PenaltyReason::UNPENALISED && previousState.penalised) {
						currentState.penalised = false;
						emit(std::move(std::make_unique<messages::output::Say>("Unpenalised")));
						
					}

					// Am I in my zone?
					try {
						currentState.selfInZone = ZONES.at(MY_ZONE).zone.pointContained(currentState.position);
					}

					catch (const std::domain_error& e) {
						std::cerr << "pointContained failed." << std::endl;
						std::cerr << "selfPosition - (" << currentState.position[0] << ", " << currentState.position[1] << ")" << std::endl;
						std::cerr << "MY_ZONE - (" << MY_ZONE << std::endl;
					}

					// Can I see the ball?
					currentState.ballSeen = ((currentState.ball.sr_xx < BALL_CERTAINTY_THRESHOLD) && (currentState.ball.sr_xy < BALL_CERTAINTY_THRESHOLD) && (currentState.ball.sr_yy < BALL_CERTAINTY_THRESHOLD));

					// Is the ball lost?
					currentState.ballLost = !currentState.ballSeen;

					// Has the ball moved?
					currentState.ballHasMoved = arma::norm(currentState.ball.position - previousState.ball.position, 2) > BALL_MOVEMENT_THRESHOLD;

					// Is the ball in my zone?
					currentState.ballInZone = !currentState.ballLost && ZONES.at(MY_ZONE).zone.pointContained(globalBallPosition);

					// Are the goals in range?
					// x = 0 = centre field.
					// Assumption: We could potentially kick a goal if we are in the other half of the field.
					// Assumption: goal.position is the x, y coordinates of the goals relative to us.
					currentState.goalInRange = (!currentState.ballLost && (arma::norm(currentState.ball.position, 2) < MAX_BALL_DISTANCE) && (globalBallPosition[0] > 0));

					// Perform calculations to see if we have reached the assigned target position and heading.
					arma::vec2 selfToPoint = currentState.targetHeading - currentState.position;
					arma::vec2 selfRotation = currentState.heading - currentState.position;

					double selfToPointAngle = std::atan2(selfToPoint[1], selfToPoint[0]);
					double selfAngle = std::atan2(selfRotation[1], selfRotation[0]);

					currentState.correctHeading = std::fabs(utility::math::angle::normalizeAngle(selfAngle - selfToPointAngle)) < ANGLE_THRESHOLD;
					currentState.inPosition = arma::norm(currentState.position - currentState.targetPosition, 2) < POSITION_THRESHOLD_TIGHT;

					currentState.outOfPosition = (arma::norm(currentState.position - currentState.targetPosition, 2) >= POSITION_THRESHOLD_LOOSE) && previousState.inPosition;

					// Am I in position to kick the ball?
//					bool kickThreshold = arma::norm(currentState.ball.position, 2) < KICK_DISTANCE_THRESHOLD;
//					bool kickThreshold = arma::norm(currentState.ball.position, 2) < KICK_DISTANCE_THRESHOLD;

//					currentState.kickPosition = (currentState.inPosition && currentState.correctHeading && kickThreshold && !currentState.outOfPosition);
					currentState.kickPosition = true;

					// If the balls position, relative to us is (0, 0) then the ball is inside us.
					// If the balls velocity is (0, 0) then it can not be approaching anything.
					if (((currentState.ball.position[0] == 0) && (currentState.ball.position[1] == 0)) || ((currentState.ball.velocity[0] == 0) && (currentState.ball.velocity[1] == 0))) {
						currentState.ballApproaching = false;
						currentState.ballApproachingGoal = false;
					}

					else {
						// Make preparations to calculate whether the ball is approaching our own goals or ourselves.
						Plane<2> planeGoal, planeSelf;
						ParametricLine<2> line;
						arma::vec2 xaxis = {1, 0};
						arma::vec2 fieldWidth = {-FIELD_DESCRIPTION.dimensions.field_length / 2, 0};

						try {
							planeGoal.setFromNormal(xaxis, fieldWidth);
						}

						catch (const std::domain_error& e) {
							std::cerr << "xaxis - (" << xaxis[0] << ", " << xaxis[1] << ")" << std::endl;
							std::cerr << "fieldWidth - (" << fieldWidth[0] << ", " << fieldWidth[1] << ")" << std::endl;
						}


						try {
//							planeSelf.setFromNormal(globalBallPosition - currentState.position, currentState.position);
							planeSelf.setFromNormal(currentState.ball.position, currentState.position);
						}

						catch (const std::domain_error& e) {
							std::cerr << "ballPosition - (" << currentState.ball.position[0] << ", " << currentState.ball.position[1] << ")" << std::endl;
							std::cerr << "selfPosition - (" << currentState.position[0] << ", " << currentState.position[1] << ")" << std::endl;
//							std::cerr << "normal - (" << (globalBallPosition - currentState.position)[0] << ", " << (globalBallPosition - currentState.position)[1] << ")" << std::endl;
							std::cerr << "normal - (" << currentState.ball.position[0] << ", " << currentState.ball.position[1] << ")" << std::endl;
						}


						try {
							line.setFromDirection(arma::vec(utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.velocity)) - arma::vec(currentState.position), arma::vec(globalBallPosition));
						}

						catch (const std::domain_error& e) {
							std::cerr << "ballVelocity - (" << currentState.ball.velocity[0] << ", " << currentState.ball.velocity[1] << ")" << std::endl;
							std::cerr << "ballPosition - (" << globalBallPosition[0] << ", " << globalBallPosition[1] << ")" << std::endl;
						}


						// Is the ball approaching our goals?
						try {
							// Throws std::domain_error if there is no intersection.
							currentState.ballGoalIntersection = planeGoal.intersect(line);
							currentState.ballApproachingGoal = arma::norm(fieldWidth - currentState.ballGoalIntersection, 2) <= (FIELD_DESCRIPTION.dimensions.goal_area_width / 2);
						}

						catch (const std::domain_error& e) {
							currentState.ballApproachingGoal = false;
						}


						// Is the ball heading in my direction?
						try {
							// Throws std::domain_error if there is no intersection.
							currentState.ballSelfIntersection = planeSelf.intersect(line);
							currentState.ballApproaching = arma::norm(currentState.position - currentState.ballSelfIntersection, 2) <= (BALL_SELF_INTERSECTION_REGION / 2);
						}

						catch (const std::domain_error& e) {
							currentState.ballApproaching = false;
						}

					}

					// Calculate the optimal zone position.
					arma::vec2 optimalPosition = findOptimalPosition(ZONES.at(MY_ZONE).zone, globalBallPosition);


					// ------
					// Take appropriate action depending on state
					// ------

					// Stop moving if in the initial, ready or finished states, as well as when picked up
					if ((currentState.primaryGameState == GameStatePrimary::INITIAL) || (currentState.primaryGameState == GameStatePrimary::SET) || (currentState.primaryGameState == GameStatePrimary::FINISHED) || currentState.pickedUp) {
						stopMoving();
					}

					// Stop moving and try to localise when penalised
					else if(currentState.penalised) {
						stopMoving();
						findSelf();
					}

					// Move to the start position if in set state
					else if (currentState.primaryGameState == GameStatePrimary::READY) {
						arma::vec2 heading = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};

						if (isWalking && currentState.inPosition && currentState.correctHeading) {
							stopMoving();
						}

						if (!isWalking && (!currentState.inPosition || !currentState.correctHeading)) {
std::cerr << "selfPosition: (" << currentState.position[0] << ", " << currentState.position[1] << ")" << std::endl;
std::cerr << "targetPosition: (" << currentState.targetPosition[0] << ", " << currentState.targetPosition[1] << ")" << std::endl;
std::cerr << "GoToPoint(startPosition): (" << ZONES.at(MY_ZONE).startPosition[0] << ", " << ZONES.at(MY_ZONE).startPosition[1] << ")" << std::endl;
							goToPoint(ZONES.at(MY_ZONE).startPosition, heading);
						}
					}

					// Move to optimal position within zone, if not in zone
					else if (!currentState.selfInZone) {
						arma::vec2 heading = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};

						if (currentState.ballLost) {
							optimalPosition = ZONES.at(MY_ZONE).defaultPosition;
						}

std::cerr << "selfPosition: (" << currentState.position[0] << ", " << currentState.position[1] << ")" << std::endl;
std::cerr << "GoToPoint(optimalPosition): (" << optimalPosition[0] << ", " << optimalPosition[1] << ")" << std::endl;

						// Only emit a goToPoint command when we are changing our destination.
						if ((optimalPosition[0] != previousState.targetPosition[0]) || (optimalPosition[1] != previousState.targetPosition[1])) {
							goToPoint(optimalPosition, heading);
						}

						findBall();
					}

					else if ((currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK) && IS_GOALIE && currentState.ballLost) {
						findBall();

//						NUClear::log<NUClear::INFO>("Penalty kick in progress. Locating ball.");
					}

					else if ((currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK) && IS_GOALIE && !currentState.ballLost && currentState.ballHasMoved && !currentState.ballApproachingGoal) {
						arma::vec2 blockPosition = {currentState.position[0], globalBallPosition[1]};
						sideStepToPoint(blockPosition);

					}

					else if ((currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK) && currentState.ballLost && currentState.kicker) {
						findBall();

//						NUClear::log<NUClear::INFO>("Penalty kick in progress. Locating ball.");
					}

					else if (currentState.secondaryGameState == GameStateSecondary::PENALTY_KICK && !currentState.ballLost && currentState.kicker) {
						arma::vec2 goal = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
						approachBall(goal);
						kickBall(goal);

//						NUClear::log<NUClear::INFO>("Penalty kick in progress. Approaching ball.");
					}

					else if ((previousState.primaryGameState == GameStatePrimary::SET) && (currentState.primaryGameState == GameStatePrimary::PLAYING) && currentState.kickOff && currentState.kicker) {
//						kickBall(currentState.heading);
						arma::vec2 goal = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
						kickBall(goal);

//						NUClear::log<NUClear::INFO>("Game just started. Time to kick off.");
					}

					else if (isKicking) {
						watchBall();

//						NUClear::log<NUClear::INFO>("I be looking at what I be kicking.");
					}

					else if (currentState.ballLost) {
						findBall();

//						NUClear::log<NUClear::INFO>("Don't know where the ball is. Looking for it.");
					}

					else if ((currentState.ballInZone || currentState.ballApproaching) && currentState.goalInRange) {
						arma::vec2 goal = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
						arma::vec2 target = utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.position);

						if ((target[0] != previousState.targetPosition[0]) || (target[1] != previousState.targetPosition[1])) {
							approachBall(goal);
						}

//						NUClear::log<NUClear::INFO>("Walking to ball.");
					}

					else if ((currentState.ballInZone || currentState.ballApproaching) && !currentState.goalInRange) {
						arma::vec2 nearestZone = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0}; // Find the optimal point in the nearest zone, reflect the position closer to the enemy goal.
						arma::vec2 target = utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.position);

						if ((target[0] != previousState.targetPosition[0]) || (target[1] != previousState.targetPosition[1])) {
							//approachBall(nearestZone);
							arma::vec2 goal = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
							approachBall(goal);
						}

//						NUClear::log<NUClear::INFO>("Walking to ball.");
					}

					else if (currentState.kickPosition && !isKicking) {
//						kickBall(currentState.heading);
						arma::vec2 goal = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
						kickBall(goal);

//						NUClear::log<NUClear::INFO>("In kicking position. Kicking ball.");
					}

					else if (currentState.goalInRange && !isKicking) {
						arma::vec2 goal = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
						arma::vec2 target = utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.position);

						if ((target[0] != previousState.targetPosition[0]) || (target[1] != previousState.targetPosition[1])) {
							approachBall(goal);
						}

						kickBall(goal);

//						NUClear::log<NUClear::INFO>("Kick for goal.");
					}

					else if (IS_GOALIE && currentState.ballApproachingGoal) {
						sideStepToPoint(currentState.ballGoalIntersection);

//						NUClear::log<NUClear::INFO>("Ball is approaching goal. Goalie moving to block it.");
					}

					else {
						arma::vec2 heading = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
						findSelf();

						if ((ZONES.at(MY_ZONE).defaultPosition[0] != previousState.targetPosition[0]) || (ZONES.at(MY_ZONE).defaultPosition[1] != previousState.targetPosition[1])) {
							goToPoint(ZONES.at(MY_ZONE).defaultPosition, heading);
						}

//						NUClear::log<NUClear::INFO>("Unknown behavioural state. Finding self, finding ball, moving to default position.");
					}

				});
			}

			void SoccerStrategy::updateGameState(const messages::input::gameevents::GameState& gameState) {
				// What state is the game in?
				switch (gameState.phase) {
					case Phase::READY:
						currentState.primaryGameState = GameStatePrimary::READY;
						if(currentState.primaryGameState != previousState.primaryGameState) {
							emit(std::move(std::make_unique<messages::output::Say>("ready")));
							std::cerr << "ready" << std::endl;
						}
						break;

					case Phase::SET:
						currentState.primaryGameState = GameStatePrimary::SET;
						if(currentState.primaryGameState != previousState.primaryGameState) {
							emit(std::move(std::make_unique<messages::output::Say>("set")));
							std::cerr << "set" << std::endl;
						}
						break;

					case Phase::PLAYING:
						currentState.primaryGameState = GameStatePrimary::PLAYING;
						if(currentState.primaryGameState != previousState.primaryGameState) {
							emit(std::move(std::make_unique<messages::output::Say>("playing")));
							std::cerr << "playing" << std::endl;
						}
						break;

					case Phase::TIMEOUT:
						currentState.primaryGameState = GameStatePrimary::TIMEOUT;
						if(currentState.primaryGameState != previousState.primaryGameState) {
							emit(std::move(std::make_unique<messages::output::Say>("timeout")));
							std::cerr << "timeout" << std::endl;
						}
						break;

					case Phase::FINISHED:
						currentState.primaryGameState = GameStatePrimary::FINISHED;
						if(currentState.primaryGameState != previousState.primaryGameState) {
							emit(std::move(std::make_unique<messages::output::Say>("finished")));
							std::cerr << "finished" << std::endl;
						}
						break;

					case Phase::INITIAL:
					default:
						currentState.primaryGameState = GameStatePrimary::INITIAL;
						if(currentState.primaryGameState != previousState.primaryGameState) {
							emit(std::move(std::make_unique<messages::output::Say>("initial")));
							std::cerr << "initial" << std::endl;
						}
						break;
				}

				switch (gameState.mode) {
					case messages::input::gameevents::Mode::PENALTY_SHOOTOUT:
						currentState.secondaryGameState = GameStateSecondary::PENALTY_SHOOTOUT;
						if(currentState.secondaryGameState != previousState.secondaryGameState) {
							emit(std::move(std::make_unique<messages::output::Say>("penalty shoot out")));
							std::cerr << "penalty shootout" << std::endl;
						}
						break;

					case messages::input::gameevents::Mode::OVERTIME:
						currentState.secondaryGameState = GameStateSecondary::OVERTIME;
						if(currentState.secondaryGameState != previousState.secondaryGameState) {
							emit(std::move(std::make_unique<messages::output::Say>("over time")));
							std::cerr << "overtime" << std::endl;
						}
						break;

					case messages::input::gameevents::Mode::NORMAL:
					default:
						currentState.secondaryGameState = GameStateSecondary::NORMAL;
						if(currentState.secondaryGameState != previousState.secondaryGameState) {
							emit(std::move(std::make_unique<messages::output::Say>("normal")));
							std::cerr << "normal" << std::endl;
						}
						break;
				}
			}

			arma::vec2 SoccerStrategy::findOptimalPosition(const Polygon& zone, const arma::vec2& point) {
				try {
					return(zone.projectPointToPolygon(point));
				}

				catch (const std::domain_error& e) {
					return(ZONES.at(MY_ZONE).defaultPosition);
				}
			}

			void SoccerStrategy::stopMoving() {
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();

				approach->targetPositionType = WalkTarget::WayPoint;
				approach->targetHeadingType = WalkTarget::WayPoint;
				approach->heading = currentState.heading;
				approach->target = currentState.position;
				approach->walkMovementType = WalkApproach::StandStill;
				emit(std::move(approach));
			}

			void SoccerStrategy::findSelf() {
				/* Try to locate both goals. */
				/* Look at closest goal for short period to reset localisation. */

				// Alternate looking at the goals and looking at the ball.
				if (!lookingAtGoal) {
std::cerr << "Emitting LookAtGoalStart" << std::endl;
					emit(std::make_unique<LookAtGoalStart>());
				}

				else {
std::cerr << "Emitting LookAtGoalStop" << std::endl;
					emit(std::make_unique<LookAtGoalStop>());
				}

				if (!lookingAtBall && !lookingAtGoal) {
std::cerr << "Emitting LookAtBallStart" << std::endl;
					emit(std::make_unique<LookAtBallStart>());
				}

				else {
std::cerr << "Emitting LookAtBallStop" << std::endl;
					emit(std::make_unique<LookAtBallStop>());
				}

//				arma::vec2 goalPosition = {FIELD_DESCRIPTION.dimensions.field_length / 2, 0};
//				arma::vec2 ballPosition = utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.position);

				// Look at our goals.
//				auto lookAtGoal = std::make_unique<messages::behaviour::LookAtPoint>();
//				lookAtGoal->x = -goalPosition[0];
//				lookAtGoal->y = goalPosition[1];
//				lookAtGoal->xError = GOAL_LOOK_ERROR[0];
//				lookAtGoal->yError = GOAL_LOOK_ERROR[1];
//				emit(std::move(lookAtGoal));

				// Look at enemy goals.
//				lookAtGoal->x = goalPosition[0];
//				lookAtGoal->y = goalPosition[1];
//				lookAtGoal->xError = GOAL_LOOK_ERROR[0];
//				lookAtGoal->yError = GOAL_LOOK_ERROR[1];
//				emit(std::move(lookAtGoal));

				// Look for ball.
			}

			void SoccerStrategy::findBall() {
//				if (lookingAtGoal) {
//					emit(std::make_unique<LookAtGoalStop>());
//				}

				if (!lookingAtBall) {
					emit(std::make_unique<LookAtBallStart>());
				}

				// This needs to be replaced with a proper lookAtBall command.
//				arma::vec2 ballPosition = utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.position);

//				auto look = std::make_unique<messages::behaviour::LookAtPoint>();
//				look->x = ballPosition[0];
//				look->y = ballPosition[1];
//				look->xError = BALL_LOOK_ERROR[0];
//				look->yError = BALL_LOOK_ERROR[1];
//				emit(std::move(look));
			}

			void SoccerStrategy::goToPoint(const arma::vec2& position, const arma::vec2& heading) {
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
				approach->targetPositionType = WalkTarget::WayPoint;
				approach->targetHeadingType = WalkTarget::WayPoint;
				approach->walkMovementType = WalkApproach::WalkToPoint;
				approach->heading = heading;
				approach->target = position;

				currentState.targetPosition = approach->target;
				currentState.targetHeading = approach->heading;

				emit(std::move(approach));
			}

			void SoccerStrategy::watchBall() {
				if (lookingAtGoal) {
					emit(std::make_unique<LookAtGoalStop>());
				}

				if (!lookingAtBall) {
					emit(std::make_unique<LookAtBallStart>());
				}

				// This needs to be replaced with a proper lookAtBall command.
//				arma::vec2 ballPosition = utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.position);

//				auto look = std::make_unique<messages::behaviour::LookAtPoint>();
//				look->x = ballPosition[0];
//				look->y = ballPosition[1];
//				look->xError = BALL_LOOK_ERROR[0];
//				look->yError = BALL_LOOK_ERROR[1];
//				emit(std::move(look));
				// This needs to be replaced with a proper lookAtBall command.
			}

			void SoccerStrategy::sideStepToPoint(const arma::vec2& position) {
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
				approach->targetPositionType = WalkTarget::WayPoint;
				approach->targetHeadingType = WalkTarget::Ball;
				approach->walkMovementType = WalkApproach::OmnidirectionalReposition;
				approach->heading = utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.position);
				approach->target = position;

				currentState.targetPosition = approach->target;
				currentState.targetHeading = approach->heading;

				emit(std::move(approach));
			}

			void SoccerStrategy::approachBall(const arma::vec2& heading) {
				auto approach = std::make_unique<messages::behaviour::WalkStrategy>();
				approach->targetPositionType = WalkTarget::Ball;
				approach->targetHeadingType = WalkTarget::WayPoint;
				approach->walkMovementType = WalkApproach::ApproachFromDirection;
				approach->heading = heading;
				approach->target = utility::localisation::transform::RobotToWorldTransform(currentState.position, currentState.heading, currentState.ball.position);

				currentState.targetPosition = approach->target;
				currentState.targetHeading = approach->heading;

				emit(std::move(approach));
			}

			void SoccerStrategy::kickBall(const arma::vec2& direction) {
				// Emit aim vector.
				if (!isKicking) {
					stopMoving();

					//emit(std::move(std::make_unique<messages::output::Say>("Emit kick command")));
					auto kick = std::make_unique<messages::behaviour::KickPlan>();
					kick->target = direction;
					emit(std::move(kick));
				}
			}
		}  // strategy
	}  // behaviours
}  // modules
