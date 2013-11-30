/*
 * This file is part of AudioInput.
 *
 * AudioInput is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * AudioInput is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with AudioInput.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_SOUNDCHUNK_H
#define MESSAGES_SOUNDCHUNK_H

#include <nuclear>
#include <vector>
#include <cstdint>

namespace messages {
    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    struct SoundChunkSettings {
        /// The number of samples that are taken each second for the sound chunks
        size_t sampleRate;
        /// The number of channels that the sound chunks will have
        size_t channels;
        /// The number of frames (a frame is a single sample for all channels) that each emitted chunk will have
        size_t chunkSize;
    };
    
    
    struct SoundFileStart {

        std::string fileName;
        NUClear::clock::time_point time;
    };

    /**
     * TODO document
     *
     * @author Jake Woods
     */
    struct SoundChunk {

        NUClear::clock::time_point endTime;
        std::vector<int16_t> data;
    };
}

#endif  // MESSAGES_SOUNDCHUNK_H

