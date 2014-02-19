/*
 * This file is part of Darwin Hardware IO.
 *
 * Darwin Hardware IO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Hardware IO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Hardware IO.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Darwin.h"

#include <thread>
#include <algorithm>

namespace Darwin {
    // Initialize all of the sensor handler objects using the passed uart
    Darwin::Darwin(const char* name) : uart(name)
    , cm730(uart, ID::CM730)
    , rShoulderPitch(uart, ID::R_SHOULDER_PITCH)
    , lShoulderPitch(uart, ID::L_SHOULDER_PITCH)
    , rShoulderRoll(uart, ID::R_SHOULDER_ROLL)
    , lShoulderRoll(uart, ID::L_SHOULDER_ROLL)
    , rElbow(uart, ID::R_ELBOW)
    , lElbow(uart, ID::L_ELBOW)
    , rHipYaw(uart, ID::R_HIP_YAW)
    , lHipYaw(uart, ID::L_HIP_YAW)
    , rHipRoll(uart, ID::R_HIP_ROLL)
    , lHipRoll(uart, ID::L_HIP_ROLL)
    , rHipPitch(uart, ID::R_HIP_PITCH)
    , lHipPitch(uart, ID::L_HIP_PITCH)
    , rKnee(uart, ID::R_KNEE)
    , lKnee(uart, ID::L_KNEE)
    , rAnklePitch(uart, ID::R_ANKLE_PITCH)
    , lAnklePitch(uart, ID::L_ANKLE_PITCH)
    , rAnkleRoll(uart, ID::R_ANKLE_ROLL)
    , lAnkleRoll(uart, ID::L_ANKLE_ROLL)
    , headPan(uart, ID::HEAD_YAW)
    , headTilt(uart, ID::HEAD_PITCH)
    , rFSR(uart, ID::R_FSR)
    , lFSR(uart, ID::L_FSR)
    , rMissile(uart, ID::R_MISSILE)
    , lMissile(uart, ID::L_MISSILE) {

        // Turn on the dynamixel power
        cm730.turnOnDynamixel();

        // Wait about 300ms for the dynamixels to start up
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // Build our bulk read packet
        buildBulkReadPacket();

        // Now that the dynamixels should have started up, set their delay time to 0 (it may not have been configured before)
        uart.execute(DarwinDevice::WriteCommand<uint8_t>(ID::BROADCAST, CM730::Address::RETURN_DELAY_TIME, 0));
    }

    std::vector<std::pair<uint8_t, bool>> Darwin::selfTest() {

        std::vector<std::pair<uint8_t, bool>> results;

        // Ping our CM730
        results.push_back(std::make_pair(ID::CM730, cm730.ping()));

        // Ping all our servos
        for (int i = 0; i < 20; ++i) {
            results.push_back(std::make_pair(i + 1, (&rShoulderPitch)[i].ping()));
        }

        // Ping our two FSRs
        results.push_back(std::make_pair(ID::L_FSR, lFSR.ping()));
        results.push_back(std::make_pair(ID::R_FSR, rFSR.ping()));

        return results;
    }

    void Darwin::buildBulkReadPacket() {

        // Double check that our type is big enough to hold the result
        static_assert(sizeof(Types::CM730Data) == CM730::Address::VOLTAGE - CM730::Address::LED_PANNEL + 1,
                      "The CM730 type is the wrong size");

        // Double check that our type is big enough to hold the result
        static_assert(sizeof(Types::MX28Data) == MX28::Address::PRESENT_TEMPERATURE - MX28::Address::TORQUE_ENABLE + 1,
                      "The MX28 type is the wrong size");

        // Double check that our type is big enough to hold the result
        static_assert(sizeof(Types::FSRData) == FSR::Address::FSR_Y - FSR::Address::FSR1_L + 1,
                      "The FSR type is the wrong size");



        // Do a self test so that we can move all the sensors that are currently failing to the end of the list
        std::vector<std::pair<uint8_t, bool>> sensors = selfTest();
        std::sort(std::begin(sensors), std::end(sensors), [](const std::pair<uint8_t, bool>& a, const std::pair<uint8_t, bool>& b) {
            return a.second > b.second;
        });

        // This holds our request paramters
        std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> request;

        // We run through our sorted list, this way any failing sensors are put at the back of the list
        for (const auto& sensor : sensors) {
            switch (sensor.first) {

                // If it's the CM730
                case ID::CM730:
                    request.push_back(std::make_tuple(CM730::Address::LED_PANNEL, ID::CM730, sizeof(Types::CM730Data)));
                    break;

                // If it's the FSRs
                case ID::L_FSR:
                case ID::R_FSR:
                    //request.push_back(std::make_tuple(FSR::Address::FSR1_L, sensor.first, sizeof(Types::FSRData)));
                    break;

                // Otherwise we assume that it's a servo
                default:
                    request.push_back(std::make_tuple(MX28::Address::TORQUE_ENABLE, sensor.first, sizeof(Types::MX28Data)));
                    break;
            }
        }

        // Our actual packet we are sending
        std::vector<uint8_t> command(7 + (request.size() * 3));
        command[Packet::MAGIC] = 0xFF;
        command[Packet::MAGIC + 1] = 0xFF;
        command[Packet::ID] = ID::BROADCAST;
        command[Packet::LENGTH] = 3 + (request.size() * 3);
        command[Packet::INSTRUCTION] = DarwinDevice::Instruction::BULK_READ;
        command[Packet::PARAMETER] = 0x00;

        // Copy our parameters in
        memcpy(&command[Packet::PARAMETER + 1], request.data(), request.size() * 3);

        // Calculate our checksum
        command.back() = calculateChecksum(command.data());

        // Swap our command into the actual command location
        bulkReadCommand.swap(command);
    }

    BulkReadResults Darwin::bulkRead() {

        // Execute the BulkRead command
        std::vector<CommandResult> results = uart.executeBulk(bulkReadCommand);

        BulkReadResults data;

        bool firstError = true;

        for (size_t i = 0; i < results.size(); ++i) {

            auto& r = results[i];

            // If we got data back
            if (!r.data.empty()) {

                // Copy for servo data
                if(r.header.id >= ID::R_SHOULDER_PITCH && r.header.id <= ID::HEAD_PITCH) {
                    memcpy(&data.servos[r.header.id - 1], r.data.data(), sizeof(Types::MX28Data));
                    data.servoErrorCodes[r.header.id - 1] = r.header.errorcode;
                }

                // Copy for FSR data
                else if(r.header.id == ID::R_FSR || r.header.id == ID::L_FSR) {
                    memcpy(data.fsr + (r.header.id - ID::R_FSR), r.data.data(), sizeof(Types::FSRData));
                    data.fsrErrorCodes[r.header.id - ID::R_FSR] = r.header.errorcode;
                }

                // Copy CM730 data
                else if(r.header.id == ID::CM730) {
                    memcpy(&data, r.data.data(), sizeof(Types::CM730Data));
                    data.cm730ErrorCode = r.header.errorcode;
                }
            }
            // If we got an error that caused us to have no data
            else {

                // We only move the first error code to the end of the list
                if (firstError && i != results.size() - 1) {

                    uint8_t bytes[3];
                    memcpy(bytes, &bulkReadCommand[Packet::PARAMETER + 1 + (i * 3)], 3);

                    // Erase our 3 bytes for this packet
                    bulkReadCommand.erase(std::begin(bulkReadCommand) + Packet::PARAMETER + 1 + (i * 3),
                                          std::begin(bulkReadCommand) + Packet::PARAMETER + 1 + (i * 3) + 3);

                    // Insert our 3 bytes at the end
                    bulkReadCommand.insert(std::end(bulkReadCommand) - 1, bytes, bytes + 3);

                    firstError = false;
                }

                // Set for servo data
                if (r.header.id >= ID::R_SHOULDER_PITCH && r.header.id <= ID::HEAD_PITCH) {
                    memset(&data.servos[r.header.id - 1], 0xFF, sizeof(Types::MX28Data));
                    data.servoErrorCodes[r.header.id - 1] = r.header.errorcode;
                }

                // Set for FSR data
                else if (r.header.id == ID::R_FSR || r.header.id == ID::L_FSR) {
                    memset(&data.fsr[r.header.id - ID::R_FSR], 0xFF, sizeof(Types::FSRData));
                    data.fsrErrorCodes[r.header.id - ID::R_FSR] = r.header.errorcode;
                }

                // Set CM730 data
                else if (r.header.id == ID::CM730) {
                    memset(&data, 0xFF, sizeof(Types::CM730Data));
                    data.cm730ErrorCode = r.header.errorcode;
                }
            }
        }

        return data;
    }

    DarwinDevice& Darwin::operator [](int id) {
        switch (id) {
            case ID::CM730:
                return cm730;
            case ID::R_SHOULDER_PITCH:
                return rShoulderPitch;
            case ID::L_SHOULDER_PITCH:
                return lShoulderPitch;
            case ID::R_SHOULDER_ROLL:
                return rShoulderRoll;
            case ID::L_SHOULDER_ROLL:
                return lShoulderRoll;
            case ID::R_ELBOW:
                return rElbow;
            case ID::L_ELBOW:
                return lElbow;
            case ID::R_HIP_YAW:
                return rHipYaw;
            case ID::L_HIP_YAW:
                return lHipYaw;
            case ID::R_HIP_ROLL:
                return rHipRoll;
            case ID::L_HIP_ROLL:
                return lHipRoll;
            case ID::R_HIP_PITCH:
                return rHipPitch;
            case ID::L_HIP_PITCH:
                return lHipPitch;
            case ID::R_KNEE:
                return rKnee;
            case ID::L_KNEE:
                return lKnee;
            case ID::R_ANKLE_PITCH:
                return rAnklePitch;
            case ID::L_ANKLE_PITCH:
                return lAnklePitch;
            case ID::R_ANKLE_ROLL:
                return rAnkleRoll;
            case ID::L_ANKLE_ROLL:
                return lAnkleRoll;
            case ID::HEAD_YAW:
                return headPan;
            case ID::HEAD_PITCH:
                return headTilt;
            case ID::R_FSR:
                return rFSR;
            case ID::L_FSR:
                return lFSR;
            default:
                throw std::runtime_error("Unknown device id");
        }
    }

    void Darwin::writeServos(const std::vector<Types::ServoValues>& servos) {

        // Check that our ServoValues object is the correct size (the difference + 1 + another for the id)
        static_assert(sizeof(Types::ServoValues) == MX28::Address::MOVING_SPEED_H - MX28::Address::D_GAIN + 2,
                      "The ServoValues type is the wrong size");

        // We allocate 8 bytes for normal things, and then space for all the servo values
        std::vector<uint8_t> packet;
        packet.resize(8 + (servos.size() * sizeof(Types::ServoValues)));

        // Build our packet
        packet[Packet::MAGIC] = 0xFF;
        packet[Packet::MAGIC + 1] = 0xFF;
        packet[Packet::ID] = ID::BROADCAST; // Broadcast id
        packet[Packet::LENGTH] = 4 + (servos.size() * sizeof(Types::ServoValues));
        packet[Packet::INSTRUCTION] = DarwinDevice::Instruction::SYNC_WRITE;

        // Our data length (not including our ID)
        packet[Packet::PARAMETER] = MX28::Address::D_GAIN;
        packet[Packet::PARAMETER + 1] = sizeof(Types::ServoValues) - 1;

        // Our motor values
        memcpy(&packet[Packet::PARAMETER + 2], servos.data(), servos.size() * sizeof(Types::ServoValues));

        // Our checksum
        packet.back() = calculateChecksum(packet.data());

        // Execute the command
        uart.executeBroadcast(packet);
    }
}
