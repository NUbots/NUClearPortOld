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

#ifndef MODULES_INPUT_CAMCALLBACKS_H
#define MODULES_INPUT_CAMCALLBACKS_H

#include <map>
#include <memory>
#include <string>
#include <flycapture/FlyCapture2.h>
#include "PtGreyCamera.h"
#include "FlycapCamera.h"


#include "messages/input/Image.h"
//#include "V4L2CameraSetting.h"

namespace modules {
    namespace input {
        

        //these are used for radial lenses with a circular display area to speed up the image demosaicing
        //get the factor of 2 aligned left edge of the circle
        static constexpr size_t getViewStart(const int& ptHeight, const int& width, const int& height, const int& radius) {
            return static_cast<size_t>(((width/2 - (int)sqrt(radius*radius - (ptHeight-height/2)*(ptHeight-height/2))) / 2) * 2);
        }
        
        //get the factor of 2 aligned right edge of the circle
        static constexpr size_t getViewEnd(const int& ptHeight, const int& width, const int& height, const int& radius) {
            return static_cast<size_t>(((width/2 + (int)sqrt(radius*radius - (ptHeight-height/2)*(ptHeight-height/2))) / 2) * 2);
        }        


        
        /**
         * @brief This class encapsulates the physical camera device. It will setup a camera device and begin streaming
         *    images
         *
         * @details
         *    This callback function converts a radial lens Bayer format image to rgb to work with NUClear.
         *
         * @author Josiah Walker
         */
        void captureRadial(FlyCapture2::Image* pImage, const void* pCallbackData);

    }  // input
}  // modules

#endif  // MODULES_INPUT_CAMCALLBACKS_H

