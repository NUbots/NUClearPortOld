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

#ifndef MESSAGES_PLATFORM_DARWIN_DARWINSENSORS_H
#define MESSAGES_PLATFORM_DARWIN_DARWINSENSORS_H

#include <nuclear>
#include <cstdint>
#include <string>
#include <stdexcept>

#include "messages/input/ServoID.h"

namespace messages {
    namespace platform {
        namespace darwin {

            /**
             * TODO document
             *
             * @author Trent Houliston
             */
            struct DarwinSensors {

    			// bitmask values
    	        enum Error {
    	            OK              = 0, // not really a flag but the lack of any other flag
    	            INPUT_VOLTAGE   = 1 << 0,
    	            ANGLE_LIMIT     = 1 << 1,
    	            OVERHEATING     = 1 << 2,
    	            RANGE           = 1 << 3,
    	            CHECKSUM        = 1 << 4,
    	            OVERLOAD        = 1 << 5,
    	            INSTRUCTION     = 1 << 6,
    	            CORRUPT_DATA    = 1 << 7,
    	            TIMEOUT         = 1 << 8
    	        };

                NUClear::clock::time_point timestamp;

                uint8_t cm730ErrorFlags;

                struct LEDPanel {
                    bool led2;
                    bool led3;
                    bool led4;
                } ledPanel;

                struct ColouredLED {
                    uint8_t r;
                    uint8_t g;
                    uint8_t b;
                };

                struct HeadLED : public ColouredLED {
                } headLED;

                struct EyeLED : public ColouredLED {
                } eyeLED;

                struct Buttons {
                    bool left;
                    bool middle;
                } buttons;

                float voltage;

		        struct Accelerometer {
		            float x;
		            float y;
		            float z;
		        } accelerometer;

                struct Gyroscope {
                    float x;
                    float y;
                    float z;
                } gyroscope;

                struct FSR {
                    float fsr1;
                    float fsr2;
                    float fsr3;
                    float fsr4;

                    float centreX;
                    float centreY;

                    uint8_t errorFlags;
                };

                struct FSRs {
                    FSR left;
                    FSR right;
                } fsr;

                struct Servo {

                    uint8_t errorFlags;

                    bool torqueEnabled;
                    bool led;

                    float pGain;
                    float iGain;
                    float dGain;

                    float goalPosition;
                    float movingSpeed;
                    float torqueLimit;

                    float presentPosition;
                    float presentSpeed;

                    float load;
                    float voltage;
                    uint8_t temperature;
                };

                struct Servos {
                    Servo rShoulderPitch;
                    Servo lShoulderPitch;
                    Servo rShoulderRoll;
                    Servo lShoulderRoll;
                    Servo rElbow;
                    Servo lElbow;
                    Servo rHipYaw;
                    Servo lHipYaw;
                    Servo rHipRoll;
                    Servo lHipRoll;
                    Servo rHipPitch;
                    Servo lHipPitch;
                    Servo rKnee;
                    Servo lKnee;
                    Servo rAnklePitch;
                    Servo lAnklePitch;
                    Servo rAnkleRoll;
                    Servo lAnkleRoll;
                    Servo headPan;
                    Servo headTilt;

                    const Servo& operator[](messages::input::ServoID index) const;
                    Servo& operator[](messages::input::ServoID index);

                    const Servo& operator[](int index) const;
                    Servo& operator[](int index);

                } servo;
            };
            
        }  // darwin
    }  // platform
}  // messages

#endif  // MESSAGES_PLATFORM_DARWIN_DARWINSENSORS_H
