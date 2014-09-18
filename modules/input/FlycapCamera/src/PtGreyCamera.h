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

#ifndef MODULES_INPUT_PTGREYCAMERA_H
#define MODULES_INPUT_PTGREYCAMERA_H

#include <map>
#include <memory>
#include <string>
#include <flycapture/FlyCapture2.h>
#include <yaml-cpp/yaml.h>

#include "messages/input/Image.h"
#include "messages/support/Configuration.h"
//#include "V4L2CameraSetting.h"

namespace modules {
    namespace input {
        using messages::support::Configuration;

        /**
         * @brief This class encapsulates the physical camera device. It will setup a camera device and begin streaming
         *    images
         *
         * @details
         *    This class uses the Video4Linux2 kernel drivers in order to connect to and get data from the darwins built in
         *    webcam. It allocates 2 kernel mode buffers which are mapped into user space. These buffers are alternated and
         *  the data from the most recently filled one is used to construct an image object. This class also provides easy
         *  access to all the settings that are available on the camera. It is provided as a map in order to make accessing
         *  the paramters by name (from a config file) easier
         *
         * @author Michael Burton
         * @author Jake Woods
         * @author Trent Houliston
         */
        class PtGreyCamera {
        private:
            
            
            std::pair<void*,PtGreyCamera*> callbackArgs;

            /// @brief the width of the image being retrieved from the camera
            size_t width;

            /// @brief the height of the image being retrieved from the camera
            size_t height;

            /// @brief this map is used to hold several ioctl wrappers that let us set settings easily
            std::map<std::string, FlyCapture2::Property> settings;


            /// @brief the format that we are reading in from the camera
            std::string format;

            /// @brief Whether the camera is currently in streaming mode
            bool streaming;

        public:
            /// @brief The name of the device to read camera data from
            int deviceID;
            
            /// @brief Our internal camera class that interacts with the physical device
            FlyCapture2::Camera camera;
            
            /// @brief this enum holds important constants (we are c++ we don't use defines for this kind of thing)
            enum {
                /// @brief the framerate we are requesting
                FRAMERATE = 15
            };

            /**
             * @brief Constructs a new DarwinCamera class using the passed string as the device
             *
             * @param device the path to the video device to use (e.g. /dev/video0)
             */
            PtGreyCamera();

            /**
             * @brief Gets a pointer to the latest image from the camera so that it can be sent out to the rest of the system
             *
             * @details
             *   This function blocks until the camera device provides a new frame of video
             *   data, at which point it copies the frame into a new Image and returns. The
             *   camera device must already be set up (using resetCamera) and
             *
             * @return a pointer to the latest image from the camera
             */
            std::unique_ptr<messages::input::Image> getImage();

            /**
             * @brief Sets up the camera at a given resolution
             *
             * @param name the name of the camera device
             * @param w the image's width
             * @param h the image's height
             * @param f whether the camera is mounted upside down
             */
            void resetCamera(const size_t device, size_t w, size_t h);

            /**
             * @brief Returns a map of all configurable settings
             */
            std::map<std::string, FlyCapture2::Property>& getSettings();

            /**
             * @brief Returns the horizontal resolution the camera is currently set to
             */
            size_t getWidth() const;

            /**
             * @brief Returns the vertical resolution the camera is currently set to
             */
            size_t getHeight() const;

            /**
             * @brief Returns the device id that is currently used as the camera
             */
            const int& getDeviceID() const;

            /**
             * @brief returns the format that the camera is currently reading (YUYV or MJPG)
             */
            const std::string& getFormat() const;

            /**
             * @brief This method is to be called when shutting down the system. It does cleanup on the cameras resources
             */
            void closeCamera();

            /**
             * @brief Starts the camera streaming video
             */
            void startStreaming(void* reactor);

            /**
             * @brief Check whether the camera is actively streaming video
             */
            bool isStreaming() const;

            /**
             * @brief Stops the camera streaming video
             */
            void stopStreaming();
            
            /**
             * @brief reconfigures the camera parameters
             */
            void configure(const YAML::Node& config);
        };

    }  // input
}  // modules

#endif  // MODULES_INPUT_V4L2CAMERA_H
