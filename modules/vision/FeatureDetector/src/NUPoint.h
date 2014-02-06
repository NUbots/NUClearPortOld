/*
 * This file is part of NUBots FeatureDetector.
 *
 * NUBots FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_NUPOINT_H
#define MODULES_VISION_NUPOINT_H

#include <nuclear>
#include <armadillo>
#include <ostream>

namespace modules {
    namespace vision {

        typedef struct {
            arma::vec2 screenCartesian;
            arma::vec2 screenAngular;
            arma::vec2 groundCartesian;
            arma::vec3 neckRelativeRadial;
        } NUPoint;
    }
}

#endif // MODULES_VISION_NUPOINT_H
