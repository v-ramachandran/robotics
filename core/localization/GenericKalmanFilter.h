/*
 * KalmanFilter.h
 *
 *  Created on: Oct 4, 2015
 *      Author: venket
 */

#ifndef CORE_LOCALIZATION_GKALMANFILTER_H_
#define CORE_LOCALIZATION_GKALMANFILTER_H_
#include<Eigen/Core>
#include<Eigen/LU>
using namespace Eigen;

template<int Dimensions>
class GenericKalmanFilter {
  public:
    virtual void reset() = 0;
    virtual Matrix<float, Dimensions, 1> apply(Eigen::Matrix<float, Dimensions, 1> newMeasurement);
  protected:
    virtual Matrix<float, Dimensions, 1> calculateMeasurement(Matrix<float, Dimensions, 1> newMeasurement, Matrix<float, Dimensions, 1> measurement) = 0;
    Matrix<float,Dimensions,1> state;
    Matrix<float,Dimensions,1> measurement;
    Matrix<float,Dimensions,Dimensions> covariance;
    Matrix<float,Dimensions,Dimensions> A, C, R, Q;
};

class SimpleBallFilter : public GenericKalmanFilter<6> {
  public:
	  SimpleBallFilter();
    void reset() override;  	
    Matrix<float, 6, 1> calculateMeasurement(Matrix<float, 6, 1> newMeasurement, Matrix<float, 6, 1> measurement) override;
    Matrix<float, 6, 1> apply(Eigen::Matrix<float, 6, 1> newMeasurement) override;
};

#endif /* CORE_LOCALIZATION_KALMANFILTER_H_ */
