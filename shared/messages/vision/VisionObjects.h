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

#ifndef MESSAGES_VISION_VISIONOBJECTS_H
#define MESSAGES_VISION_VISIONOBJECTS_H

#include <armadillo>
#include <nuclear>

#include "messages/vision/ClassifiedImage.h"

namespace messages {
    namespace vision {

        class VisionObject {
        public:
            VisionObject() {}

            arma::vec3 sphericalFromNeck;   //bodyRelativeSpherical
            arma::vec3 sphericalError;
            arma::vec2 screenAngular;   //Polar around view vector on image
            arma::vec2 screenCartesian;
            arma::vec2 sizeOnScreen;
            NUClear::clock::time_point timestamp;
        };

        class Ball : public VisionObject {
        public:
            Ball() : VisionObject() {}
            float diameter;
        };

        class Goal : public VisionObject {
        public:
            Goal() : VisionObject() {}
            enum Type{
                LEFT,
                RIGHT,
                UNKNOWN
            } type;

            //Order convention: tr, br, bl, tl,
            std::vector<arma::vec2> screen_quad;
        };

        class Obstacle : public VisionObject {
        public:
            Obstacle() : VisionObject() {}
            float arcWidth;
            // enum ColourType{
            //  TEAM_CYAN,
            //  TEAM_MAGENTA,
            //  UNKNOWN
            // }
            ColourClass colour;
        };

        //Line objects:

        class FieldLine : public VisionObject {
        public:
            FieldLine() : VisionObject() {}
        };

        class CornerPoint : public VisionObject {
        public:
            CornerPoint() : VisionObject() {}

            enum Type {
                L_CORNER,
                T_CORNER,
                X_CORNER,
                INVALID
            } type;
        };

        class CentreCircle : public VisionObject {
        public:
            CentreCircle() : VisionObject() {}
        };

        class LineObjects{
        public:
            LineObjects() {}
            std::vector<CentreCircle> centre_circles;
            std::vector<CornerPoint> corner_points;
            std::vector<FieldLine> field_lines;
        };

    }
}

#endif // MESSAGES_VISION_VISIONOBJECTS_H
