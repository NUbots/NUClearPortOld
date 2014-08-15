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


#include "utility/image/ColorModelConversions.h"
#include "messages/input/Image.h"
#include "messages/input/CameraParameters.h"
#include "messages/support/Configuration.h"

namespace modules {
    namespace input {

        using messages::input::Image;

        PtGreyCamera::PtGreyCamera() : width(0), height(0), deviceID(-1), streaming(false) {
            
        }

        std::unique_ptr<Image> PtGreyCamera::getImage() {
            if (!streaming) {
                return nullptr;
            }
            /*
            v4l2_buffer current;
            memset(&current, 0, sizeof(current));
            current.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            current.memory = V4L2_MEMORY_MMAP;

            // Get our frame buffer with data in it
            if (ioctl(fd, VIDIOC_DQBUF, &current) == -1) {
                throw std::system_error(errno, std::system_category(), "There was an error while de-queuing a buffer");
            }

            std::vector<Image::Pixel> data(width * height);
            std::unique_ptr<Image> image;

             {
                uint8_t* input = static_cast<uint8_t*>(buff[current.index].payload);

                const size_t total = width * height;

                // Fix the colour information to be YUV444 rather then YUV422
                for(size_t i = 0; i < total; ++++i) {

                    data[i].y  = input[i * 2];
                    data[i].cb = input[i * 2 + 1];
                    data[i].cr = input[i * 2 + 3];

                    data[i + 1].y  = input[i * 2 + 2];
                    data[i + 1].cb = input[i * 2 + 1];
                    data[i + 1].cr = input[i * 2 + 3];
                }

                // Move this data into the image
                image = std::unique_ptr<Image>(new Image(width, height, std::move(data)));
            }*/
            
            // Return our image
            std::unique_ptr<Image> image;
            return image;
        }

        void PtGreyCamera::resetCamera(const size_t device, size_t w, size_t h) {
            // if the camera device is already open, close it
            closeCamera();
            
            //XXX: select devices properly
            FlyCapture2::Error error = camera.Connect( 0 );
            
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

        void PtGreyCamera::startStreaming() {
            if (!streaming) {
                // Start streaming data
                FlyCapture2::Error error = camera.StartCapture();
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

        void PtGreyCamera::closeCamera() {
            if (deviceID != -1) {
                stopStreaming();
                camera.Disconnect();
                deviceID = -1;
            }
        }

    }  // input
}  // modules
