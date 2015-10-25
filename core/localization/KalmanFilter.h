/*
 * KalmanFilter.h
 *
 *  Created on: Oct 4, 2015
 *      Author: venket
 */

#ifndef CORE_LOCALIZATION_KALMANFILTER_H_
#define CORE_LOCALIZATION_KALMANFILTER_H_
#include<Eigen/Core>
#include<Eigen/LU>
using namespace Eigen;

template<int Dimensions>
class KalmanFilter {
  public:
    virtual void reset() = 0;
    virtual Matrix<float,Dimensions,1> specificFunction(Matrix<float,Dimensions,1> newMeasurement) = 0;
  protected:
    Matrix<float,Dimensions,1> state;
    Matrix<float,Dimensions,1> measurement;
    Matrix<float,Dimensions,Dimensions> covariance;
    Matrix<float,Dimensions,Dimensions> A, C, R, Q;

  /* ... */
};

class BallFilter : public KalmanFilter<6> {
  public:
	  BallFilter();
    void reset();  	
    Matrix<float,6,1> specificFunction(Matrix<float,6,1> newMeasurement) override;
};

#endif /* CORE_LOCALIZATION_KALMANFILTER_H_ */
