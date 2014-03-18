#ifndef IMGLSH_H
#define IMGLSH_H

#include <armadillo>
#include <cmath>
#include <iostream>
#include <random>
#include <iterator>
#include <algorithm>

//XXX: should be moved to messages
struct {
    uint32_t top,
    uint32_t left,
    uint32_t bottom,
    uint32_t right,S
    size_t objClass,
    double votes,
} ImageLabel;


template <size_t NEIGHBOURS = 1, size_t TABLES = 50, size_t BITS = 12>
class ImageHasher {
private:
    //hash functions and tables
    std::vector<SimpleHash3D<BITS>> hashFns;
    LSHTable<NEIGHBOURS,TABLES,BITS> tables;
    
    //this is the size of the image to hash
    size_t windowSizeX;
    size_t windowSizeY;
    
    //this is how far to move the hash window each step
    size_t strideX;
    size_t strideY;
    
    std::vector<arma::cube> tableData;
    std::vector<uint16_t> tableLabels;
    
    //heap based distance sorting - much faster than naive sorting as we don't evaluate half the queries a lot of the time
    std::vector<std::pair<uint16_t,double>> distSortCandidates (const arma::cube& img, const size_t& x, const size_t& y, const std::set<uint16_t>& candidates) {
        
        //make a result vector
        std::vector<std::pair<uint16_t,double>> results;
        results.reserve(NEIGHBOURS);
        
        //get the subslice of the image that we are interested in
        const auto& m = img.submat(x,y,x+windowSizeX,y+windowSizeY);
        
        //create a comparator for the heap
        const auto cmp = [](const std::pair<size_t, double>& a, const std::pair<size_t, double>& b) {
            return a.second < b.second;
        };
        
        //find closest candidates
        for (auto& c : candidates) {
            const double dist = sqrt(arma::accu(m%tabledata[c]));
            if (results.size < NEIGHBOURS) {
                //just insert until the heap us full
                results.emplace_back(tableLabels[c],dist);
                std::push_heap(results.begin(),resuts.end());
            } else if (results[0].second > dist) {
                //replace if this neighbour is better
                std::pop_heap(results.begin(),resuts.end());
                results[NEIGHBOURS-1] = std::make_pair(tableLabels[c],dist);
                std::push_heap(results.begin(),resuts.end());
            }
        }
        
        //sort candidates
        const auto cmp2 = [](const std::pair<size_t, double>& a, const std::pair<size_t, double>& b) {
            return a.second > b.second;
        };
        std::sort(candidates.begin(),candidates.end(),cmp2);
        
        return candidates;
    }
    

public:
    
    ImageHasher(size_t windowX, size_t windowY, size_t strideX, size_t strideY) : 
        windowSizeX(windowX), windowSizeY(windowY), 
        strideX(strideX), strideY(strideY) {
        ;
    }
    
    void insert(const std::vector<arma::cube>& queryData, const std::vector<uint16_t>& queryLabels) {
        arma::mat qhashes = arma::mat(TABLES,queryData.size);
        for (size_t i = 0; i < TABLES; ++i) {
            for (size_t j = 0; j < queryData.size; ++j) {
                qhashes(i,j) = hashFns[i].getHash(queryData[j]);
            }
        }
        tables.insert(qhashes);
        tableData = queryData;
        tableLabels = queryLabels;
    }
    
    std::vector<ImageLabel> query(const arma::cube& image) {
        std::vector<ImageLabel> imageCandidates;
        imageCandidates.reserve(20);
        std::vector<uint16_t> currentHashes;
        currentHashes.resize(TABLES);
        for (size_t i = 0; i < image.n_rows-windowSizeX; i+=strideX) {
            for (size_t j = 0; j < image.n_cols-windowSizeY; j+=strideY) {
                
                //generate all the hashes for a patch
                for (size_t k = 0; k < TABLES; ++k) {
                    currentHashes[k] = hashFns[i].getHash(image,i,j);
                }
                
                //do the lookups and figure out what to label the patch
                std::set<uint16_t> candidates = tables.query(currentHashes);
                if (candidates.size > 0) {
                    std::vector<std::pair<uint16_t,double>> result = distSortCandidates(image,i,j,candidates);
                    
                    //store the result candidate for later processing
                    imageCandidates.emplace_back(i,j,i+windowSizeX,j+windowSizeY,results[0].first,results[0].second)
                }
            }
        }
        return imageCandidates;
    }
}

#endif
