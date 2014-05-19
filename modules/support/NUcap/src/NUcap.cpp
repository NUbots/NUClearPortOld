/*
 * This file is part of NUbots Codebase.
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

#include "NUcap.h"
#include <nuclear>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <netdb.h>

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>

#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "messages/support/Configuration.h"

#include "messages/input/proto/MotionCapture.pb.h"

namespace modules {
namespace support {

    using messages::support::Configuration;
    using messages::input::proto::MotionCapture;

    NUcap::NUcap(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<NUcap> > >([this](const Configuration<NUcap>& config) {

            uint32_t serverIP = inet_addr(config["serverIP"].as<std::string>().c_str());
            uint32_t clientIP = inet_addr(config["clientIP"].as<std::string>().c_str());

            NUClear::log("serverIP", serverIP);
            NUClear::log("clientIP", clientIP);
            

            // Version number of the NatNet protocol, as reported by the server.
            unsigned char natNetMajor;
            unsigned char natNetMinor;

            // Use this socket address to send commands to the server.
            struct sockaddr_in serverCommands = NatNet::createAddress(serverIP, NatNet::commandPort);

            // Create sockets
            sdCommand = NatNet::createCommandSocket( clientIP );
            sdData = NatNet::createDataSocket( clientIP );

            // Start the CommandListener in a new thread.
            commandListener = std::make_unique<CommandListener>(sdCommand);
            commandListener->start();

            // Send a ping packet to the server so that it sends us the NatNet version
            // in its response to commandListener.
            NatNetPacket ping = NatNetPacket::pingPacket();
            ping.send(sdCommand, serverCommands);

            // Wait here for ping response to give us the NatNet version.
            commandListener->getNatNetVersion(natNetMajor, natNetMinor);

            // Start up a FrameListener in a new thread.
            frameListener = std::make_unique<FrameListener>(sdData, natNetMajor, natNetMinor);
            frameListener->start();

        });

        on<Trigger<Every<1, std::chrono::milliseconds>>>([this](const time_t&) {
            bool valid;
            // Try to get a new frame from the listener.
            MocapFrame frame(frameListener->pop(&valid).first);
            // Quit if the listener has no more frames.
            if (!valid) {
                return;
            }
//            std::cout << frame << std::endl;

            auto moCap = std::make_unique<MotionCapture>();

            for (auto marker : frame.markerSets()) {
                auto* markerSet = moCap->add_marker_sets();

                markerSet->set_name(marker.name());
                for (auto point : marker.markers()) {
                    auto* marker_point = markerSet->add_points();
                    marker_point->set_x(point.x);
                    marker_point->set_y(point.y);
                    marker_point->set_z(point.z);
                }
            }

            for (auto point : frame.unIdMarkers()) {
                auto* marker_point = moCap->add_unidentified_points();
                NUClear::log("Point", point.x, point.y, point.z);
                marker_point->set_x(point.x);
                marker_point->set_y(point.y);
                marker_point->set_z(point.z);
            }

            for (auto fRigidBody : frame.rigidBodies()) {
                auto* rigidBody = moCap->add_rigid_bodies();
                rigidBody->set_identifier(std::to_string(fRigidBody.id()));

                auto rotation = fRigidBody.orientation();

                rigidBody->set_qw(rotation.qw);
                rigidBody->set_qx(rotation.qx);
                rigidBody->set_qy(rotation.qy);
                rigidBody->set_qz(rotation.qz);
                // TODO: fRigidBody.location();
                for (auto point : fRigidBody.markers()) {
                    auto* marker_point = rigidBody->add_points();
                    marker_point->set_x(point.x);
                    marker_point->set_y(point.y);
                    marker_point->set_z(point.z);
                }
            }

            emit<Scope::NETWORK>(std::move(moCap));

        });

        on<Trigger<Shutdown>>([this](const Shutdown&) {
            // Wait for threads to finish.
            frameListener->stop();
            commandListener->stop();
            frameListener->join();
            commandListener->join();

            // Epilogue
            close(sdData);
            close(sdCommand);
        });
    }

}
}

