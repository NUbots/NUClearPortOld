/*
 * This file is part of LSHvision.
 *
 * LUTClassifier is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LUTClassifier is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LUTClassifier.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Josiah Walker <josiah.walker@uon.edu.au>
 */

#include "LSHvision.h"
#include "ImageHasher.h"

namespace modules {
    namespace research {

		using messages::input::Image;
		using messages::support::Configuration;
		using utility::configuration::ConfigurationNode;

		using std::chrono::system_clock;
        
        LSHvision::LSHvision(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), greenHorizon(), scanLines() { 
			currentLUTIndex = 0;
			
            /*on<Trigger<Configuration<VisionConstants>>>([this](const Configuration<VisionConstants>& constants) {
           		//std::cout<< "Loading VisionConstants."<<std::endl;
           		//std::cout<< "Finished Config Loading successfully."<<std::endl;
            });*/

            on<Trigger<Image>, With<Raw<Image>>, Options<Single>>([this](const Image& image, const std::shared_ptr<const Image>& image_ptr) {
            	
            	arma::Cube<uint8_t> img(image_ptr, Image.y, 3, Image.x, copy_aux_mem = true, strict = true);
            	//XXX: threshold image into a cube
            	

            	//emit(std::move(classifiedImage));
            });

        }

    }  // vision
}  // modules
