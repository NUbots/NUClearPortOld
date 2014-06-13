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

#include "LinuxCamera.h"
#include "utility/idiom/pimpl_impl.h"

extern "C" {
    #include <jpeglib.h>
}

#include "V4L2Camera.h"
#include "messages/input/Image.h"
#include "messages/support/Configuration.h"

namespace modules {
    namespace input {

        using messages::support::Configuration;

        // Create our impl class as per the pimpl idiom.
        class LinuxCamera::impl {
            public:
                /// @brief Our internal camera class that interacts with the physical device
                V4L2Camera camera;
        };


        // We assume that the device will always be video0, if not then change this
        LinuxCamera::LinuxCamera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // This trigger gets us as close as we can to the frame rate as possible (as high resolution as we can)
            on<Trigger<Every<NUClear::clock::period::den / V4L2Camera::FRAMERATE, NUClear::clock::duration>>, Options<Single>>([this](const time_t&) {

                // If the camera is ready, get an image and emit it
                if (m->camera.isStreaming()) {
                    emit(m->camera.getImage());
                }
            });

            // When we shutdown, we must tell our camera class to close (stop streaming)
            on<Trigger<Shutdown>>([this](const Shutdown&) {
                m->camera.closeCamera();
            });

            on<Trigger<Configuration<LinuxCamera>>>([this](const Configuration<LinuxCamera>& config) {
                auto& camera = m->camera;

                try {
                    // Recreate the camera device at the required resolution
                    int width = config["imageWidth"].as<int>();
                    int height = config["imageHeight"].as<int>();
                    std::string deviceID = config["deviceID"].as<std::string>();
                    std::string format = config["imageFormat"].as<std::string>();
                    bool rotated = config["rotated"].as<bool>();

                    if (camera.getWidth() != static_cast<size_t>(width)
                        || camera.getHeight() != static_cast<size_t>(height)
                        || camera.getFormat() != format
                        || camera.getDeviceID() != deviceID) {
                        camera.resetCamera(deviceID, format, width, height, rotated);
                    }

                    // Set all other camera settings
                    for(auto& setting : camera.getSettings()) {
                        int value = config[setting.first].as<int>();
                        if(setting.second.set(value) == false) {
                            NUClear::log<NUClear::DEBUG>("Failed to set " + setting.first + " on camera");
                        }
                    }

                    // Start the camera streaming video
                    m->camera.startStreaming();
                } catch(const std::exception& e) {
                    NUClear::log<NUClear::DEBUG>(std::string("Exception while setting camera configuration: ") + e.what());
                }
            });
        }

    }  // input
}  // modules
