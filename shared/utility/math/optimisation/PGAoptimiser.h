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
 * Copyright 2014 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_PGAOPTIMISER_H
#define UTILITY_MATH_PGAOPTIMISER_H

#include <armadillo>
#include <cmath>

namespace utility {
    namespace math {
        namespace optimisation {
            
            /**
             * Function to generate a new best-estimate using the parameter samples and fitnesses provided.
             *
             * Takes a row-wise list of sample parameters, a corresponding vector of fitnesses, and a selectivity constant c (don't change c unless you know what it does)
             * @author Josiah Walker
             */
            arma::vec PGAupdateEstimate(const arma::mat& samples, const arma::vec& fitnesses, const double c = 7.0) {
                
                //create a vector of normed fitnesses
                const double min = arma::min(fitnesses);
                const double max = arma::max(fitnesses);
                const arma::vec normedFitnesses = (max-fitnesses)/(max-min+std::numeric_limits<double>::epsilon());
                
                //create a set of weights per sample which specifies the likelihood that they are near the best estimate
                const arma::vec sampleWeights = arma::exp(-c*normedFitnesses);
                
                //return the probabilistically weighted result estimate
                return arma::sum(samples % arma::repmat(sampleWeights/arma::accu(sampleWeights),1,samples.n_cols),0).t();
            }

            /**
             * Function to create a new set of parameter samples from a best estimate and
             * a vec of per-dimension scales for the gaussian noise additive (sigmaweights).
             *
             * @author Josiah walker
             */
            arma::mat PGAgetSamples(const arma::vec& bestEstimate, const arma::vec& sigmaWeights, const size_t& numSamples) {
                return   arma::randn<arma::mat>(numSamples,bestEstimate.n_elem) 
                       % arma::repmat(sigmaWeights, 1, numSamples).t() 
                       + arma::repmat(bestEstimate, 1, numSamples).t();
            }
        
        }
    }
}


#endif // UTILITY_MATH_COORDINATES_H

