#ifndef HASHERS_H
#define HASHERS_H

#include <armadillo>
#include <cmath>
#include <iostream>
#include <random>
#include <iterator>
#include <algorithm>

/**
 * @class SimpleHash3D
 *
 * Creates a hash from a 3d thresholded source matrix, using offsets to capture different windows.
 *
 * Author: Josiah Walker
 */
 template <size_t BITS = 12>
 class SimpleHash3D {
 private:
    
    struct {
        uint16_t x;
        uint16_t y;
        uint16_t channel;
    } hashIdx;
    
    
    std::vector<hashIdx> hash;
 
 public:
    
    SimpleHash3D (const size_t& windowSizeX, const size_t& windowSizeY) {
        hash.clear();
        
        //generate all possible indices
        for (size_t i = 0; i < windowSizeX; ++i) {
            for (size_t j = 0; j < windowSizeY; ++j) {
                for (size_t k = 0; k < 3; ++k) {
                    hash.emplace_back(i,j,k);
                }
            }
        }
        
        //do a shuffle
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(hash.begin(), hash.end(), g);
        
        //keep only the ones we need
        hash.resize(BITS);
    }
    
    uint16_t getHash(const arma::cube& qry,const size_t offsetX = 0,const size_t offsetY = 0) {
        uint16_t result = 0;
        
        for (size_t i = 0; i < BITS; ++i) {
            const auto& h = hash[i];
            result += qry[h.x+offsetX][h.y+offsetY][h.channel] << i;
        }
        
        return result;
    }
 
 }
