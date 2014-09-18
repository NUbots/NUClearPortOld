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

#include "FlycapCamera.h"

extern "C" {
    #include <jpeglib.h>
}
#include "utility/image/ColorModelConversions.h"
#include "messages/input/CameraParameters.h"
#include "messages/support/Configuration.h"
#include "CamCallbacks.h"

namespace modules {
    namespace input {

        using messages::support::Configuration;
        using messages::input::CameraParameters;
        //using namespace FlyCapture2;

        // We assume that the device will always be video0, if not then change this
        FlycapCamera::FlycapCamera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // When we shutdown, we must tell our camera class to close (stop streaming)
            on<Trigger<Shutdown>>([this](const Shutdown&) {
                for (auto& camera: cameras) {
                    camera.closeCamera();
                }
            });

            on<Trigger<Configuration<FlycapCamera>>>([this](const Configuration<FlycapCamera>& config) {

                
                
                    //XXX: NOT PER CAMERA
                    auto cameraParameters = std::make_unique<CameraParameters>();

                    cameraParameters->imageSizePixels << config["imageWidth"].as<uint>() << config["imageHeight"].as<uint>();
                    cameraParameters->FOV << config["FOV_X"].as<double>() << config["FOV_Y"].as<double>();
                    cameraParameters->distortionFactor = config["DISTORTION_FACTOR"].as<double>();
                    arma::vec2 tanHalfFOV;
                    tanHalfFOV << std::tan(cameraParameters->FOV[0] * 0.5) << std::tan(cameraParameters->FOV[1] * 0.5);
                    arma::vec2 imageCentre;
                    imageCentre << cameraParameters->imageSizePixels[0] * 0.5 << cameraParameters->imageSizePixels[1] * 0.5;
                    cameraParameters->pixelsToTanThetaFactor << (tanHalfFOV[0] / imageCentre[0]) << (tanHalfFOV[1] / imageCentre[1]);
                    cameraParameters->focalLengthPixels = imageCentre[0] / tanHalfFOV[0];
                emit<Scope::DIRECT>(std::move(cameraParameters));
                
                
                    PtGreyCamera* cameraptr = nullptr;
                    
                    for (PtGreyCamera& c : cameras) {
                        std::cout << c.deviceID << std::endl;
                        if (c.deviceID == config["deviceID"].as<int>()) {
                            cameraptr = &c;
                            break;
                        } else if (c.deviceID == -1) {
                            cameraptr = &c;
                            break;
                        }
                    }
                    
                    PtGreyCamera& camera = *cameraptr;
                
                try {
                    //if thisisaradialcamera
                    camera.resetCamera(config["deviceID"].as<uint>(), 1280, 960);
                    camera.configure(config.config);
                    //camera.startStreaming();
                    //XXX: we should only StartCapture AFTER all settings are set
                    camera.startStreaming(this);
                    
                    
                } catch(const std::exception& e) {
                    NUClear::log<NUClear::DEBUG>(std::string("Exception while starting camera streaming: ") + e.what());
                    throw e;
                }
                
 });
            /*on<Trigger<Every<1, std::chrono::seconds>>, With<Configuration<LinuxCamera>>>("Camera Setting Applicator", [this] (const time_t&, const Configuration<LinuxCamera>& config) {
                if(camera.isStreaming()) {
                    // Set all other camera settings
                    for(auto& setting : camera.getSettings()) {
                        int value = config[setting.first].as<int>();
                        if(setting.second.set(value) == false) {
                            NUClear::log<NUClear::DEBUG>("Failed to set " + setting.first + " on camera");
                        }
                    }
                }
            });*/
        }

    }  // input
}  // modules
