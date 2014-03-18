#ifndef LOOKUPTABLES_H
#define LOOKUPTABLES_H

#include <armadillo>
#include <cmath>
#include <iostream>

/**
 * @class LSHTable
 *
 * Creates a LSH multi-value multi-table.
 *
 * Author: Josiah Walker
 */
template <size_t NEIGHBOURS = 1, size_t TABLES = 50, size_t BITS = 12>
class LSHTable
{
private:
    std::vector<std::vector<uint16_t>> hashTables; //the hashtable allows direct hash index lookups
    std::vector<std::vector<uint16_t>> multiTables; //the multitable stores multiple values for each hash location
public:
    
    LSHTable () {
        hashTables.resize(TABLES);
        multiTables.resize(TABLES);
        for (size_t i = 0; i < TABLES; ++i) {
            hashTables[i].resize(1<<BITS,65535); //pre-fill with max value; we expect to only have 65534 elements
            multiTables[i].reserve(1<<BITS,65535);
        }
    }
    
    
    void insertData(const arma::mat& hashVals) {
        for (size_t i = 0; i < TABLES; ++i) {
            for (uint16_t j = 0; j < hashVals.n_cols; ++j) {
                multiTables[i][j] = hashTables[i][hashVals(i,j)];
                hashTables[i][hashVals(i,j)] = j;
            }
        } //the outer vector is of tables, the inner is of items
    }
    
    std::set<uint16_t> query(const std::vector<uint16_t>& hashes) {
        std::set<uint16_t> results;
        
        for (size_t i = 0; i < TABLES; ++i) {
            auto& qry = hashTables[i][hashes[i]];
            while (qry != 65535) {
                results.emplace(qry);
                qry = multiTables[qry];
            }
        }
        
        return results;
    }


}


/**
 * @class WTATable
 *
 * Creates a LSH multi-value multi-table using winner-take-all functionality
 *
 * Author: Josiah Walker
 */
template <NEIGHBOURS = 1, OBJCLASSES = 5, size_t TABLES = 100, size_t BITS = 12>
class WTATable
{
private:
    std::vector<std::vector<arma::uvec>> hashTables; //the hashtable allows direct hash index lookups for the object votes per category
public:
    
    LSHTable () {
        hashTables.resize(TABLES);
        for (size_t i = 0; i < TABLES; ++i) {
            hashTables[i].resize(1<<BITS,arma::zeros(OBJCLASSES));
        }
    }
    
    void insertData(const std::vector<std::vector<uint16_t>>& hashVals, const std::vector<std::vector<uint16_t>>& objectIDs) {
        for (size_t i = 0; i < TABLES; ++i) {
            for (uint16_t j = 0; j < hashVals[0].size; ++j) {
                ++hashTables[i][hashVals[j]][objectIDs[i][j]];
            }
        } //the outer vector is of tables, the inner is of items
    }
    
    arma::uvec query(const std::vector<uint16_t>& hashes) {
        arma::uvec results = arma::zeros(OBJCLASSES);
        
        for (size_t i = 0; i < TABLES; ++i) {
            results += hashTables[i][hashes[i]];
        }
        
        //XXX: this is inefficient if there are a large number of object classes
        return arma::sort_index(results).resize(NEIGHBOURS);
    }
}

#endif
