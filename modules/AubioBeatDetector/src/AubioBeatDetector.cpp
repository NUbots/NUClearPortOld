/*
 * This file is part of AubioBeatDetector.
 *
 * AudioInput is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * AubioBeatDetector is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with AubioBeatDetector.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Joshua Kearns <joshau-k@hotmail.com>
 */

#include "AubioBeatDetector.h"
#include "utility/idiom/pimpl_impl.h"

extern "C" {
    #include <aubio/aubio.h>
}

#include <chrono>
#include <ctime>
#include <vector>

#include "messages/SoundChunk.h"
#include "messages/Beat.h"

namespace modules {

    const int WINDOW_SIZE = 1024;
    const int HOP_SIZE = 512; //number of frames to input to beat detection at one time

    class AubioBeatDetector::impl {
    public:

        //Beat Detection Trigger Variables
        aubio_tempo_t* tempoTracker;
        fvec_t* outputData;
        fvec_t* inputData;

        // Settings
        size_t sampleRate;
        size_t channels;
        size_t chunkSize;
        size_t offset;
        
        bool initialized = false;
    };

    // Holds beat value without a period, used to create a Beat objects with a period by comparing the last 2 beatTime's
    struct BeatTime {
        NUClear::clock::time_point time;
    };

    AubioBeatDetector::AubioBeatDetector(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<messages::SoundChunkSettings>>([this](const messages::SoundChunkSettings& settings) {
            //std::cout << "SoundChunkSettings triggered" << std::endl;
            // Store the settings of the sound chunks
            m->sampleRate = settings.sampleRate;
            m->channels = settings.channels;
            m->chunkSize = settings.chunkSize;
            
            //std::cout << "sample rate: " <<  m->sampleRate << std::endl;
            //std::cout << "channels: " <<  m->channels << std::endl;
            //std::cout << "chunkSize: " <<  m->chunkSize << std::endl;

            // Build our audio tempo tracker  can set to (aubio_onset_kl or aubio_onset_complex onset tracking)
            m->tempoTracker = new_aubio_tempo(aubio_onset_kl, WINDOW_SIZE, HOP_SIZE, m->channels);
            m->outputData = new_fvec(HOP_SIZE, m->channels);
            m->inputData = new_fvec(HOP_SIZE, m->channels);

            m->initialized = true;
        });
        
        on<Trigger<messages::SoundChunk>>([this](const messages::SoundChunk& chunk) {
            
            //while (m->initialized == false)
            //{
            
            //}
            if (m->initialized == true)
            {
                for (size_t i = 0; i < m->chunkSize; ++i) {

                    // Write to our vector
                    size_t index = (i + m->offset) % HOP_SIZE;
                    fvec_write_sample(m->inputData, chunk.data[i * m->channels], 0, index);

                    // If we are done filling this hop chunk
                    if(index == HOP_SIZE - 1) {

                        aubio_tempo(m->tempoTracker, m->inputData, m->outputData);
                        for (int i = 1; i <= m->outputData->data[0][0]; ++i) {

                            auto beatTime = std::make_unique<BeatTime>();

                            // Work out how many samples from the end we are in total
                            const size_t position = ((i - HOP_SIZE) + (m->outputData->data[0][i]));

                            // Start at our end time and go back
                            beatTime->time = chunk.endTime - NUClear::clock::duration(int(NUClear::clock::period::den * (double(position) / m->sampleRate)));

                            emit(std::move(beatTime));
                        }
                    }
                }

                m->offset = (m->chunkSize + m->offset) % HOP_SIZE;
            }
        });

       on<Trigger<Last<2, BeatTime>>>([this](const std::vector<std::shared_ptr<const BeatTime>>& lastTwoBeats) {

           if(lastTwoBeats.size() == 2) {
                auto beat = std::make_unique<messages::Beat>();
                beat->time = lastTwoBeats[0]->time; //apparently the latest one is 0
                beat->period = lastTwoBeats[0]->time - lastTwoBeats[1]->time;
                emit(std::move(beat));
           }
           else if (lastTwoBeats.size() == 1) //First one has no period
           {
                auto beat = std::make_unique<messages::Beat>();
                beat->time = lastTwoBeats[0]->time; //apparently the latest one is 0
                beat->period = NUClear::clock::duration(0);
                emit(std::move(beat));
           }
       });

       on<Trigger<Shutdown>>([this](const Shutdown&) {
          del_aubio_tempo(m->tempoTracker);
          del_fvec(m->inputData);
          del_fvec(m->outputData);
       });
    }
}