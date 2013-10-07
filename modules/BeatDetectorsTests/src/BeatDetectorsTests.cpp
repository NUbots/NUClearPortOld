/*
 * This file is part of BeatDetectorsTests.
 *
 * AudioInput is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * BeatDetectorsTests is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with BeatDetectorsTests.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Joshua Kearns <joshau-k@hotmail.com>
 */

#include <chrono>
#include <ctime>
#include "messages/Beat.h"
#include <vector>

namespace modules {



    BeatDetectorsTests::BeatDetectorsTests(NUClear::PowerPlant* plant) : Reactor(plant) {
        

        
        on<Trigger<messages::SoundChunk>> ([this](const messages::SoundChunk& chunk) {
            
        });
 
        
    }
    
    BeatDetectorsTests::~BeatDetectorsTests()
    {

    }

}