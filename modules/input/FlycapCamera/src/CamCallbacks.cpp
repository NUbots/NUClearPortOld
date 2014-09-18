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

#include "CamCallbacks.h"
#include "PtGreyCamera.h"
#include "FlycapCamera.h"


#include "messages/input/Image.h"

namespace modules {
    namespace input {
            
        /**
         * @brief This class encapsulates the physical camera device. It will setup a camera device and begin streaming
         *    images
         *
         * @details
         *    This callback function converts a radial lens Bayer format image to rgb to work with NUClear.
         *
         * @author Josiah Walker
         */
        void captureRadial(FlyCapture2::Image* pImage, const void* pCallbackData) {
            
            std::pair<FlycapCamera*,PtGreyCamera*>* cbData = reinterpret_cast<std::pair<FlycapCamera*,PtGreyCamera*>*>(const_cast<void*>(pCallbackData));
            FlycapCamera* reactor = cbData->first; //reinterpret_cast<FlycapCamera*>(const_cast<void*>(pCallbackData));
            
            constexpr uint radius = 475;
            constexpr uint sourceWidth = 1280;
            constexpr uint sourceHeight = 960;
            constexpr uint hOffset = sourceWidth/2-radius;
            
            //the horizontal offset cuts out the black areas of the image altogether to save CPU
            std::vector<messages::input::Image::Pixel> data(sourceHeight*(radius*2), {0,0,0});
            std::unique_ptr<messages::input::Image> image;
            FlyCapture2::Image& rawImage = *pImage;

            // do a cache-coherent demosaic step
            size_t j2 = 0;
            for (size_t j = (sourceHeight/2-radius)*sourceWidth; j < (sourceHeight/2+radius)*sourceWidth; j += sourceWidth) {
                
                for (size_t i = getViewStart(j/sourceWidth,sourceWidth,sourceHeight,radius); 
                    i < getViewEnd(j/sourceWidth,sourceWidth,sourceHeight,radius)-2; i += 2) { // assume we always start on an even pixel (odd ones are nasty)
                    
                    const size_t index = i+j;
                    const size_t dIndex = i+j2-hOffset+1;
                    //do the current row
                    auto& pxNext = data[dIndex];
                    auto& pxNextNext = data[dIndex+1];
                    
                    //get the required information
                    const auto& currentBlue = *rawImage[index];
                    const auto& currentGreen = *rawImage[index+1];
                    const auto& nextBlue = *rawImage[index+2];
                    const auto& nextGreen = *rawImage[index+3];
                    
                    //demosaic red and green
                    pxNext.y = (unsigned char)(((unsigned int)currentBlue + (unsigned int)nextBlue) >> 1);
                    pxNext.cb = currentGreen;
                    pxNextNext.y = nextBlue;
                    pxNextNext.cb = (unsigned char)(((unsigned int)currentGreen + (unsigned int)nextGreen) >> 1);
                    
                    //do the row below
                    //px = data[dIndex+radius*2];
                    data[dIndex+radius*2].y = (unsigned char)(((unsigned int)currentBlue + (unsigned int)nextBlue) >> 1);
                    data[dIndex+1+radius*2].y = nextBlue;
                    
                }
                j += sourceWidth;
                j2 += radius*2;
                // do the second line
                for (size_t i = getViewStart(j/sourceWidth,sourceWidth,sourceHeight,radius); 
                    i < getViewEnd(j/sourceWidth,sourceWidth,sourceHeight,radius)-2; i += 2) { // assume we always start on an even pixel (odd ones are nasty)
                    
                    const size_t index = i+j;
                    const size_t dIndex = i+j2-hOffset+1;
                    //do the current row
                    auto& pxNext = data[dIndex];
                    auto& pxNextNext = data[dIndex+1];
                    
                    //get the required information
                    const auto& currentGreen = *rawImage[index];
                    const auto& currentRed = *rawImage[index+1];
                    const auto& nextGreen = *rawImage[index+2];
                    const auto& nextRed = *rawImage[index+3];
                    
                    //demosaic red and green
                    pxNext.cb = (unsigned char)(((unsigned int)currentGreen + (unsigned int)nextGreen) >> 1);
                    pxNext.cr = currentRed;
                    pxNextNext.cb = nextGreen;
                    pxNextNext.cr = (unsigned char)(((unsigned int)currentRed + (unsigned int)nextRed) >> 1);
                    
                    //do the row below
                    data[dIndex+radius*2].cr = currentRed;
                    data[dIndex+1+radius*2].cr = (unsigned char)(((unsigned int)currentRed + (unsigned int)nextRed) >> 1);
                    
                }
                j2 += radius*2;
            }
            image = std::make_unique<messages::input::Image>(radius*2, sourceHeight, std::move(data));
            std::cout << reactor << std::endl;
            reactor->emitImage(std::move(image));
        }

    }  // input
}  // modules


