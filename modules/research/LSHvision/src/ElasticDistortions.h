#ifndef DISTORTIONS_H
#define DISTORTIONS_H

#include <armadillo>
#include <cmath>
#include <iostream>

/**
 * @class Distortions
 *
 * Creates elastic distorted images.
 *
 * Original Source: http://www.codeproject.com/Articles/16650/Neural-Network-for-Recognition-of-Handwritten-Digi
 */
class ElasticDistortions
{
public:
    //! elastic distortion
    static constexpr double sigma = 8.0;
    //! elastic scaling (in avg px)
    static constexpr double alpha = 20.0;
    //! maximal absolute rotation
    static constexpr double rotation = 0.2;
    //! maximal horizontal scaling (percent)
    static constexpr double scaleX = 0.1;
    //! maximal vertical scaling (percent)
    static constexpr double scaleY = 0.1;

    //! has to be odd (actually it just works a bit better that way)
    static constexpr int gaussKernelSize = 21;
    arma::mat gaussKernel;
    arma::mat distX, distY; //distortionH, distortionV;
    
    arma::mat genGaussian(int size, double spread) {
        const double multiplier = 1.0/(sqrt(2.0*M_PI)*spread);
        const double expmultiplier = -1.0/(2.0*spread*spread);
        arma::vec ctr = arma::linspace<arma::vec>(0,size-1,size);
        auto tmp = arma::square(arma::repmat(ctr-size/2,1,size));
        
        return multiplier*arma::exp(expmultiplier*tmp%tmp.t());
    }
    
    arma::mat convolve(arma::mat input, arma::mat gaussian) {
        //NOTE: this assumes a square gaussian
        const int rows = input.n_rows;
        const int cols = input.n_cols;
        
        arma::mat output = arma::zeros(rows,cols);
        
        int gcs2 = gaussian.n_rows/2;
        int gk = gaussian.n_rows;
        
        for(int r = 0; r < rows; r++) {
            for(int c = 0; c < cols; c++) {
                output(r,c) = arma::accu(input.submat(
                                            std::max(r-gcs2,0),
                                            std::max(c-gcs2,0),
                                            std::min(r+gcs2,rows-1),
                                            std::min(c+gcs2,cols-1)
                                        )%gaussian.submat(
                                            std::min(std::max(gcs2-r,0),gcs2),
                                            std::min(std::max(gcs2-c,0),gcs2),
                                            gk - std::max(-(rows-r-gcs2-2),1),
                                            gk - std::max(-(cols-c-gcs2-2),1)
                                        ))/arma::accu(gaussian.submat(
                                            std::min(std::max(gcs2-r,0),gcs2),
                                            std::min(std::max(gcs2-c,0),gcs2),
                                            gk - std::max(-(rows-r-gcs2-2),1),
                                            gk - std::max(-(cols-c-gcs2-2),1)
                                        ));
            }
        }
        
        return output;
    }
    
    
    ElasticDistortions() {
        gaussKernel = genGaussian(gaussKernelSize,sigma);
    }

    void createDistortionMap(int rows, int cols) {
    
        double ratio = double(rows)/cols;
        
        //start with random matrices
        distX = arma::randu(rows,cols)*2-1;
        distY = arma::randu(rows,cols)*2-1;
        
        //convolve the matrices to smooth everything out
        distX = convolve(distX,gaussKernel);
        distY = convolve(distY,gaussKernel);
        
        //scale the elastic displacements:
        distX *= alpha/sqrt(2.0);
        distY *= alpha/sqrt(2.0);
        
        //generate other random numbers
        arma::vec scalings = arma::randu<arma::vec>(3)*2.0-1.0;
        //do x scaling
        const int rowCenter = rows / 2;
        arma::vec centerVec = (arma::linspace<arma::vec>(0,rows-1,rows)-rowCenter);
        distX += scaleX*scalings[0]*repmat(centerVec,1,cols);
        
        //do x rotation component from x
        distX -= sin(rotation*scalings[2])*repmat(centerVec,1,cols);//*rows; //*ratio;
        
        //do y rotation component from x
        distY -= sin(rotation*scalings[2])*repmat(centerVec,1,cols); //*ratio;
        
        //do y scaling
        const int colCenter = cols / 2;
        centerVec = (arma::linspace<arma::vec>(0,cols-1,cols)-colCenter);
        distY += scaleY*scalings[1]*repmat(centerVec,1,rows).t();
        
        //do y rotation component from y
        distY -= sin(rotation*scalings[2])*repmat(centerVec,1,rows).t();//*cols; //ratio;
        
        //do x rotation component from y
        distX += sin(rotation*scalings[2])*repmat(centerVec,1,rows).t(); ///ratio;
        
    }

    arma::mat applyDistortion(arma::mat& input) { //XXX: this is dirty but should work
        const int rows = input.n_rows;
        const int cols = input.n_cols;
        arma::mat instance = input;
        arma::mat kernel = genGaussian(5,0.01);
        
        int gcs2 = kernel.n_rows/2;
        int gk = kernel.n_rows;
        for(int r = 0; r < rows; r++) {
            for(int c = 0; c < cols; c++) {
                int r2 = std::min(std::max(int(r+distX(r,c)),0),rows-1);
                int c2 = std::min(std::max(int(c+distY(r,c)),0),cols-1);
                instance(r,c) = arma::accu(input.submat(
                                            std::max(r2-gcs2,0),
                                            std::max(c2-gcs2,0),
                                            std::min(r2+gcs2,rows-1),
                                            std::min(c2+gcs2,cols-1)
                                        )%kernel.submat(
                                            std::min(std::max(gcs2-r2,0),gcs2),
                                            std::min(std::max(gcs2-c2,0),gcs2),
                                            gk - std::max(-(rows-r2-gcs2-2),1),
                                            gk - std::max(-(cols-c2-gcs2-2),1)
                                        ))/arma::accu(kernel.submat(
                                            std::min(std::max(gcs2-r2,0),gcs2),
                                            std::min(std::max(gcs2-c2,0),gcs2),
                                            gk - std::max(-(rows-r2-gcs2-2),1),
                                            gk - std::max(-(cols-c2-gcs2-2),1)
                                        ));
            }
        }
        return instance;
    }
};

#endif // DISTORTIONS_H

