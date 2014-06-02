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

#include "SLAME.h"
#include <cmath>
#include "SLAMEModule.h"
#include "utility/nubugger/NUgraph.h"


namespace modules {
    namespace vision {

        using utility::vision::ORBFeatureExtractor;
        using utility::vision::MockFeatureExtractor;
        using messages::input::Image;
        using messages::localisation::Self;
        using messages::support::Configuration;
        using messages::input::Sensors;
        using utility::nubugger::graph;

        SLAME::SLAME(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), ORBModule(), MockSLAMEModule() {

            on<Trigger<Configuration<SLAME>>>([this](const Configuration<SLAME>& config) {
                std::string featureExtractorName = config["FEATURE_EXTRACTOR_TYPE"];

                if(featureExtractorName.compare("ORB") == 0){
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::ORB;
                } else if(featureExtractorName.compare("LSH") == 0) {
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::LSH;
                } else if(featureExtractorName.compare("MOCK") == 0) {
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::MOCK;
                    fakeLocalisationHandle.enable();
                } else {
                    NUClear::log<NUClear::WARN>("SLAME - BAD CONFIG STRING: Loading default ORB feature detector.");
                    FEATURE_EXTRACTOR_TYPE = FeatureExtractorType::ORB;
                }
            });
            
            on<Trigger<Configuration<MockFeatureExtractor>>>([this](const Configuration<MockFeatureExtractor>& config) {
                FAKE_LOCALISATION_PERIOD = 10;//config["FAKE_LOCALISATION_CONFIG"];
                FAKE_LOCALISATION_RADIUS = 2;//config["FAKE_LOCALISATION_RADIUS"];
                MockSLAMEModule.setParameters(config); });

            on<Trigger<Configuration<ORBFeatureExtractor>>>([this](const Configuration<ORBFeatureExtractor>& config) {
                ORBModule.setParameters(config);
            });

            on<Trigger<Image>, With<std::vector<Self>, Sensors>, Options<Single>>("SLAME", [this](const Image& image, const std::vector<Self>& selfs, const Sensors& sensors){               
                switch(FEATURE_EXTRACTOR_TYPE){
                    case (FeatureExtractorType::ORB):
                        emit(ORBModule.getSLAMEObjects(image, selfs[0], sensors));
                        break;
                    case (FeatureExtractorType::LSH):
                        break;
                    case (FeatureExtractorType::MOCK):
                        auto objects = MockSLAMEModule.getSLAMEObjects(image, selfs[0], sensors);
                        emit(graph("SLAME Objects:", objects->size()));
                        emit(std::move(objects));
                        break;
                }
            });

            fakeLocalisationHandle = on<Trigger<Every<30, std::chrono::milliseconds>>>("Fake Localisation", [this](const time_t&){
                auto selfs = std::make_unique<std::vector<Self>>(1);                
                auto& s = selfs->back();
                NUClear::clock::time_point now = NUClear::clock::now();
                NUClear::clock::duration t = now - start_time;
                s.position = arma::vec({FAKE_LOCALISATION_RADIUS * std::cos(2 * M_PI * std::chrono::duration_cast<std::chrono::milliseconds>(t).count() / double(1000*FAKE_LOCALISATION_PERIOD)), 
                                         FAKE_LOCALISATION_RADIUS * std::sin(2 * M_PI * std::chrono::duration_cast<std::chrono::milliseconds>(t).count() / double(1000*FAKE_LOCALISATION_PERIOD))});
                s.heading = arma::vec({cos(2 * M_PI * std::chrono::duration_cast<std::chrono::milliseconds>(t).count() / double(1000*FAKE_LOCALISATION_PERIOD/3)), 
                                        sin(2 * M_PI * std::chrono::duration_cast<std::chrono::milliseconds>(t).count() / double(1000*FAKE_LOCALISATION_PERIOD/3))});
                emit(graph("Current localisation:", s.position[0],s.position[1]));

                s.sr_xx = 0.01;
                s.sr_xy = 0;
                s.sr_yy = 0.01;
                emit(std::move(selfs));
            });
            fakeLocalisationHandle.disable();
            start_time = NUClear::clock::now();
            // debugHandle = on<Trigger<Every<10, Per<std::chrono::seconds>>>>([this] (const time_t& now, const Sensors& sensors) {
            //     switch(FEATURE_EXTRACTOR_TYPE){
            //         case (FeatureExtractorType::ORB):
            //             emit(ORBModule.testSLAME(sensors));
            //             break;
            //         case (FeatureExtractorType::LSH):
            //             break;
            //         case (FeatureExtractorType::MOCK):
            //             break;
            //     }
            // });
        }
    }
}
