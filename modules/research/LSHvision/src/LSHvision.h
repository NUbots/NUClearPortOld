/*
 * This file is part of LUTClassifier.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_RESEARCH_LSHVISION_H
#define MODULES_RESEARCH_LSHVISION_H

#include <nuclear> 
#include <string>
#include <armadillo>
#include <chrono>

#include "messages/input/Image.h"
#include "messages/support/Configuration.h"

namespace modules {
    namespace research {
        
        static constexpr const char* CONFIGURATION_PATH = "LSHvision.json";
        
        
        /**
         * Detects objects in a raw image using LSH
         *
         * @author Josiah Walker
         */
        class LSHvision : public NUClear::Reactor {
        private:
            std::vector<uint16_t[65536]> lshtables;
            std::vector<uint16_t[65536]> lshlists;
            std::vector<uint16_t[65536]> lshhashes;
        public:
            explicit LSHvision(std::unique_ptr<NUClear::Environment> environment);
        };
    
    }  // research
}  // modules

#endif  // MODULES_RESEARCH_LSHVISION_H

