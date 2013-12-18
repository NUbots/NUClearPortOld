/*
 * This file is part of NUBots Utility.
 *
 * NUBots Utility is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUBots Utility is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUBots Utility.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_MATRIX_H
#define UTILITY_MATH_MATRIX_H

#include <armadillo>
#include <cmath>

namespace utility {
    namespace math {

        /**
         * Some general matrix utilities (generating rotation matrices).
         *
         * @author Alex Biddulph
         */
        namespace matrix {
            inline arma::mat33 xRotationMatrix(double angle) {
                return arma::mat33({0           , 0             , 0             , 
                                    0           , cos(angle)    , -sin(angle)   , 
                                    0           , sin(angle)    , cos(angle)    });
            }

            inline arma::mat33 yRotationMatrix(double angle) {
                return arma::mat33({cos(angle)  , 0             , sin(angle)    , 
                                    0           , 0             , 0             , 
                                    -sin(angle) , 0             , cos(angle)    });
            }

            inline arma::mat33 zRotationMatrix(double angle) {
                return arma::mat33({cos(angle)  , -sin(angle)   , 0             , 
                                    sin(angle)  , cos(angle)    , 0             , 
                                    0           , 0             , 0             });
            }
        }
    }
}

#endif // UTILITY_MATH_COORDINATES_H
