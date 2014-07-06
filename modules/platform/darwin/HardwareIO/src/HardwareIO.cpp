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

#include "HardwareIO.h"
#include "Convert.h"

#include "utility/math/angle.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/motion/ServoTarget.h"
#include "messages/support/Configuration.h"


namespace modules {
namespace platform {
namespace darwin {

    using messages::platform::darwin::DarwinSensors;
    using messages::motion::ServoTarget;
    using messages::support::Configuration;

    DarwinSensors HardwareIO::parseSensors(const Darwin::BulkReadResults& data) {
        DarwinSensors sensors;

        // Timestamp when our data was taken
        sensors.timestamp = NUClear::clock::now();

        /*
         CM730 Data
         */

        // Read our Error code
        sensors.cm730ErrorFlags = data.cm730ErrorCode == 0xFF ? DarwinSensors::Error::TIMEOUT : DarwinSensors::Error(data.cm730ErrorCode);

        // LED Panel
        sensors.ledPanel = cm730State.ledPanel;

        // Head LED
        sensors.headLED = cm730State.headLED;

        // Eye LED
        sensors.eyeLED = cm730State.eyeLED;

        // Buttons
        sensors.buttons.left = Convert::getBit<1>(data.cm730.buttons);
        sensors.buttons.middle = Convert::getBit<1>(data.cm730.buttons);

        // Voltage (in volts)
        sensors.voltage = Convert::voltage(data.cm730.voltage);

        // Accelerometer (in m/s^2)
        sensors.accelerometer.x = Convert::accelerometer(data.cm730.accelerometer.x);
        sensors.accelerometer.y = Convert::accelerometer(data.cm730.accelerometer.y);
        sensors.accelerometer.z = Convert::accelerometer(data.cm730.accelerometer.z);

        // Gyroscope (in radians/second)
        sensors.gyroscope.x = Convert::gyroscope(data.cm730.gyroscope.x);
        sensors.gyroscope.y = Convert::gyroscope(data.cm730.gyroscope.y);
        sensors.gyroscope.z = Convert::gyroscope(data.cm730.gyroscope.z);

        /*
         Force Sensitive Resistor Data
         */

        // Right Sensor
        // Error
        sensors.fsr.right.errorFlags = data.fsrErrorCodes[0] == 0xFF ? DarwinSensors::Error::TIMEOUT : DarwinSensors::Error(data.fsrErrorCodes[0]);

        // Sensors
        sensors.fsr.right.fsr1 = Convert::fsrForce(data.fsr[0].fsr1);
        sensors.fsr.right.fsr2 = Convert::fsrForce(data.fsr[0].fsr2);
        sensors.fsr.right.fsr3 = Convert::fsrForce(data.fsr[0].fsr3);
        sensors.fsr.right.fsr4 = Convert::fsrForce(data.fsr[0].fsr4);

        // Centre
        sensors.fsr.right.centreX = Convert::fsrCentre(false, true, data.fsr[0].centreX);
        sensors.fsr.right.centreY = Convert::fsrCentre(false, false, data.fsr[0].centreY);

        // Left Sensor
        // Error
        sensors.fsr.left.errorFlags = data.fsrErrorCodes[1] == 0xFF ? DarwinSensors::Error::TIMEOUT : DarwinSensors::Error(data.fsrErrorCodes[1]);

        // Sensors
        sensors.fsr.left.fsr1 = Convert::fsrForce(data.fsr[1].fsr1);
        sensors.fsr.left.fsr2 = Convert::fsrForce(data.fsr[1].fsr2);
        sensors.fsr.left.fsr3 = Convert::fsrForce(data.fsr[1].fsr3);
        sensors.fsr.left.fsr4 = Convert::fsrForce(data.fsr[1].fsr4);

        // Centre
        sensors.fsr.left.centreX = Convert::fsrCentre(true, true, data.fsr[1].centreX);
        sensors.fsr.left.centreY = Convert::fsrCentre(true, false, data.fsr[1].centreY);

        /*
         Servos
         */

        for(int i = 0; i < 20; ++i) {
            // Get a reference to the servo we are populating
            DarwinSensors::Servo& servo = sensors.servo[i];

            // Error code
            servo.errorFlags = data.servoErrorCodes[i] == 0xFF ? DarwinSensors::Error::TIMEOUT : DarwinSensors::Error(data.servoErrorCodes[i]);

            // Booleans
            servo.torqueEnabled = servoState[i].torqueEnabled;

            // Gain
            servo.pGain = servoState[i].pGain;
            servo.iGain = servoState[i].iGain;
            servo.dGain = servoState[i].dGain;

            // Targets
            servo.goalPosition = servoState[i].goalPosition;
            servo.movingSpeed = servoState[i].movingSpeed;

            // Present Data
            servo.presentPosition = Convert::servoPosition(i, data.servos[i].presentPosition);
            servo.presentSpeed = Convert::servoSpeed(i, data.servos[i].presentSpeed);
            servo.load = Convert::servoLoad(i, data.servos[i].load);

            // Diagnostic Information
            servo.voltage = Convert::voltage(data.servos[i].voltage);
            servo.temperature = Convert::temperature(data.servos[i].temperature);
        }

        return sensors;
    }

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), darwin("/dev/ttyUSB0") {

        on<Trigger<Configuration<Darwin::UART>>>([this](const Configuration<Darwin::UART>& config){
            darwin.setConfig(config);
        });

        // This trigger gets the sensor data from the CM730
        on<Trigger<Every<60, Per<std::chrono::seconds>>>, Options<Single>>([this](const time_t&) {

            // Our final sensor output
            auto sensors = std::make_unique<DarwinSensors>();

            std::vector<uint8_t> command = {
                0xFF,
                0xFF,
                Darwin::ID::BROADCAST,
                0x00, // The size, fill this in later
                Darwin::DarwinDevice::Instruction::SYNC_WRITE,
                Darwin::MX28::Address::D_GAIN,
                0x08
            };

            for(uint i = 0; i < servoState.size(); ++i) {

                if(servoState[i].dirty) {

                    if(!servoState[i].torqueEnabled) {
                        servoState[i].dirty = false;

                        darwin[i + 1].write(Darwin::MX28::Address::TORQUE_ENABLE, false);
                    }
                    else {
                        // Clear our dirty flag
                        servoState[i].dirty = false;

                        // Get our goal position and speed
                        uint16_t goalPosition = Convert::servoPositionInverse(i, servoState[i].goalPosition);
                        uint16_t movingSpeed = Convert::servoSpeedInverse(i, servoState[i].movingSpeed);

                        // Add to our sync write command
                        command.insert(command.end(), {
                            uint8_t(i + 1),
                            Convert::gainInverse(servoState[i].dGain), // D Gain
                            Convert::gainInverse(servoState[i].iGain), // I Gain
                            Convert::gainInverse(servoState[i].pGain), // P Gain
                            0,                                         // Reserved
                            uint8_t(0xFF & goalPosition),              // Goal Position L
                            uint8_t(0xFF & (goalPosition >> 8)),       // Goal Position H
                            uint8_t(0xFF & movingSpeed),               // Goal Speed L
                            uint8_t(0xFF & (movingSpeed >> 8))         // Goal Speed H
                        });
                    }
                }
            }

            // Write our data (if we need to)
            if(command.size() > 7) {
                // Calculate our length
                command[Darwin::Packet::LENGTH] = command.size() - 3;

                // Do a checksum
                command.push_back(0);
                command.back() = Darwin::calculateChecksum(command.data());

                darwin.sendRawCommand(command);
            }

            // Read our data
            Darwin::BulkReadResults data = darwin.bulkRead();

            // Parse our data
            *sensors = parseSensors(data);

            // Send our nicely computed sensor data out to the world
            emit(std::move(sensors));
        });

        // This trigger writes the servo positions to the hardware
        on<Trigger<std::vector<ServoTarget>>, With<DarwinSensors>>([this](const std::vector<ServoTarget>& commands, const DarwinSensors& sensors) {

            // Loop through each of our commands
            for (const auto& command : commands) {

                // If gain is 0, do a normal write to disable torque (syncwrite won't write to torqueEnable)
                if(isnan(command.gain)) {
                    // Update our internal state
                    if(servoState[uint(command.id)].torqueEnabled) {
                        servoState[uint(command.id)].dirty = true;
                        servoState[uint(command.id)].torqueEnabled = false;
                    }
                }

                // Otherwise write the command using sync write
                else {
                    float diff = utility::math::angle::difference(command.position, sensors.servo[command.id].presentPosition);
                    NUClear::clock::duration duration = command.time - NUClear::clock::now();

                    float speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));


                    // Update our internal state
                    servoState[uint(command.id)].torqueEnabled = true;

                    if(servoState[uint(command.id)].pGain != command.gain
                    || servoState[uint(command.id)].iGain != command.gain * 0
                    || servoState[uint(command.id)].dGain != command.gain * 0
                    || servoState[uint(command.id)].movingSpeed != speed
                    || servoState[uint(command.id)].goalPosition != command.position) {

                        servoState[uint(command.id)].dirty = true;

                        servoState[uint(command.id)].pGain = command.gain;
                        servoState[uint(command.id)].iGain = command.gain * 0;
                        servoState[uint(command.id)].dGain = command.gain * 0;

                        servoState[uint(command.id)].movingSpeed = speed;
                        servoState[uint(command.id)].goalPosition = command.position;
                    }
                }
            }
        });

        on<Trigger<ServoTarget>>([this](const ServoTarget command) {
            auto commandList = std::make_unique<std::vector<ServoTarget>>();
            commandList->push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(commandList));
        });

        // If we get a HeadLED command then write it
        on<Trigger<DarwinSensors::HeadLED>>([this](const DarwinSensors::HeadLED& led) {
            // Update our internal state
            cm730State.headLED = led;

            darwin.cm730.write(Darwin::CM730::Address::LED_HEAD_L, Convert::colourLEDInverse(led.r, led.g, led.b));
        });

        // If we get a EyeLED command then write it
        on<Trigger<DarwinSensors::EyeLED>>([this](const DarwinSensors::EyeLED& led) {
            // Update our internal state
            cm730State.eyeLED = led;

            darwin.cm730.write(Darwin::CM730::Address::LED_EYE_L, Convert::colourLEDInverse(led.r, led.g, led.b));
        });
    }
}
}
}
