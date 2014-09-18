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

#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdexcept>
#include <system_error>
#include <string>
#include <sstream>
#include <linux/videodev2.h>
#include <jpeglib.h>
#include "PtGreyCamera.h"
#include "CamCallbacks.h"


#include "utility/image/ColorModelConversions.h"
#include "messages/input/Image.h"
#include "messages/input/CameraParameters.h"
#include "messages/support/Configuration.h"

namespace modules {
    namespace input {

        using messages::input::Image;
        using messages::support::Configuration;

        PtGreyCamera::PtGreyCamera() : width(0), height(0), deviceID(-1), streaming(false) {
            
        }

        void PtGreyCamera::resetCamera(const size_t device, size_t w, size_t h) {
            // if the camera device is already open, close it
            closeCamera();
            std::cout << "beginning to create camera" << std::endl;
            //XXX: support multiple cameras
            FlyCapture2::PGRGuid deviceIdentifier;
            FlyCapture2::BusManager().GetCameraFromSerialNumber((unsigned int)device, &deviceIdentifier );
            std::cout << "retrieved device" << std::endl;
            FlyCapture2::Error error = camera.Connect( &deviceIdentifier );
            std::cout << "connected" << std::endl;
            
            if ( error != FlyCapture2::PGRERROR_OK )
            {
                throw std::system_error(errno, std::system_category(), "Failed to connect to camera, did you run as sudo?");
            }
            

            // Store our new state
            deviceID = device;
            width = w;
            height = h;


            // Here we set the "Format" of the device (the type of data we are getting)
            FlyCapture2::VideoMode format = FlyCapture2::VIDEOMODE_1280x960Y8;

            // We have to choose YUYV or MJPG here
            if(width != 1280 or height != 960) {
                throw std::runtime_error("The format must be 1280x960Y8");
            }
            
            error = camera.SetVideoModeAndFrameRate(format,
                                                    FlyCapture2::FRAMERATE_15);
            if ( error != FlyCapture2::PGRERROR_OK )
            {
                throw std::system_error(errno, std::system_category(), "Failed to set the format or framerate");
            }
            
            //insert the current camera properties for all camera settings
            settings.clear();
            FlyCapture2::Property p;
            p.type = FlyCapture2::BRIGHTNESS;
            camera.GetProperty(&p);
            settings.insert( std::make_pair("brightness",                 p) );
            
            p.type = FlyCapture2::AUTO_EXPOSURE;
            camera.GetProperty(&p);
            settings.insert( std::make_pair("auto_exposure",              p) );
            
            p.type = FlyCapture2::WHITE_BALANCE;
            camera.GetProperty(&p);
            settings.insert( std::make_pair("white_balance_temperature",  p) );
            
            p.type = FlyCapture2::GAMMA;
            camera.GetProperty(&p);
            settings.insert( std::make_pair("gamma",                      p) );
            
            p.type = FlyCapture2::PAN;
            camera.GetProperty(&p);
            settings.insert( std::make_pair("absolute_pan",               p) );
            
            p.type = FlyCapture2::TILT;
            camera.GetProperty(&p);
            settings.insert( std::make_pair("absolute_tilt",              p) );
            
            p.type = FlyCapture2::SHUTTER;
            camera.GetProperty(&p);
            settings.insert( std::make_pair("absolute_exposure",          p) );
            
            p.type = FlyCapture2::GAIN;
            camera.GetProperty(&p);
            settings.insert( std::make_pair("gain",                       p) );
            
            p.type = FlyCapture2::TEMPERATURE;
            camera.GetProperty(&p);
            settings.insert( std::make_pair("temperature",                p) );
        }

        void PtGreyCamera::startStreaming(void* reactor) {
            if (!streaming) {
                
                if (reactor != NULL) {
                    callbackArgs = {reactor, this};
                }
                
                // Start streaming data
                FlyCapture2::Error error = camera.StartCapture(
                                &captureRadial
                                ,&callbackArgs);
                if ( error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
                {
                    throw std::system_error(errno, std::system_category(), "Bandwidth exceeded");
                }
                else if ( error != FlyCapture2::PGRERROR_OK )
                {
                    throw std::system_error(errno, std::system_category(), "Failed to start image capture");
                }
                
                streaming = true;
            }
        }

        void PtGreyCamera::stopStreaming() {
            if (streaming) {
                
                FlyCapture2::Error error = camera.StopCapture();
                
                if ( error != FlyCapture2::PGRERROR_OK )
                {
                    throw std::system_error(errno, std::system_category(), "Failed to stop image capture");
                }
                
                streaming = false;
            }
        }

        bool PtGreyCamera::isStreaming() const {
            return streaming;
        }

        std::map<std::string, FlyCapture2::Property>& PtGreyCamera::getSettings() {
            return settings;
        }

        size_t PtGreyCamera::getWidth() const {
            return width;
        }

        size_t PtGreyCamera::getHeight() const {
            return height;
        }

        const int& PtGreyCamera::getDeviceID() const {
            return deviceID;
        }

        const std::string& PtGreyCamera::getFormat() const {
            return format;
        }

        void PtGreyCamera::configure(const YAML::Node& config) {
            //this is how to set properties
            //XXX: brightness is not enabled by default - do we want it? (ditto for temperature)
            auto& s = this->getSettings()["brightness"];
            //s.onOff = true;
            s.valueA = config["brightness"].as<unsigned int>();
            camera.SetProperty(&s);
            
            s = this->getSettings()["gain"];
            s.autoManualMode = config["gain_auto"].as<unsigned int>();
            s.valueA = config["gain"].as<unsigned int>();
            camera.SetProperty(&s);
            
            s = this->getSettings()["gamma"];
            s.onOff = true;
            s.valueA = config["gamma"].as<unsigned int>();
            camera.SetProperty(&s);
            
            s = this->getSettings()["absolute_exposure"];
            s.valueA = config["absolute_exposure"].as<unsigned int>();
            camera.SetProperty(&s);
            
            s = this->getSettings()["absolute_pan"];
            s.valueA = config["absolute_pan"].as<unsigned int>();
            camera.SetProperty(&s);
            
            s = this->getSettings()["absolute_tilt"];
            s.valueA = config["absolute_tilt"].as<unsigned int>();
            camera.SetProperty(&s);
            
            s = this->getSettings()["auto_exposure"];
            s.onOff = config["auto_exposure"].as<int>();
            s.absValue = config["auto_exposure_val"].as<float>();
            camera.SetProperty(&s);
            
            s = this->getSettings()["white_balance_temperature"];
            s.valueA = config["white_balance_temperature_red"].as<unsigned int>();
            s.valueB = config["white_balance_temperature_blue"].as<unsigned int>();
            s.onOff = config["auto_white_balance"].as<unsigned int>();
            camera.SetProperty(&s);
            
            //XXX: auto white balance
            //auto& s = camera.getSettings()["absolute_pan"];
            //s.valueA = config["absolute_pan"].as<unsigned int>();
            //camera.camera.SetProperty(&s);
        }

        void PtGreyCamera::closeCamera() {
            if (deviceID != -1) {
                stopStreaming();
                camera.Disconnect();
                deviceID = -1;
            }
        }

    }  // input
}  // modules
