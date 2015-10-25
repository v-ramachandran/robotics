/*
 * KalmanFilter.h
 *
 *  Created on: Oct 4, 2015
 *      Author: venket
 */

#ifndef CORE_LOCALIZATION_EKALMANFILTER_H_
#define CORE_LOCALIZATION_EKALMANFILTER_H_
#include<Eigen/Core>
#include<Eigen/LU>
#include<localization/KalmanFilter.h>
using namespace Eigen;


class ExtendedBallFilter : public KalmanFilter<6> {
  public:  
  ExtendedBallFilter();

   void reset() override;
   Matrix<float,6,1> specificFunction(Matrix<float,6,1> newMeasurement) override;
   Matrix<float,6,1> updateFunction(Matrix<float,6,1> state);
   Matrix<float,6,6> findJacobiG(Matrix<float,6,1> state);
   Matrix<float,6,1> extractionFunction(Matrix<float,6,1> state);
   Matrix<float,6,6> findJacobiH(Matrix<float,6,1> state);
   
};

#endif /* CORE_LOCALIZATION_KALMANFILTER_H_ */
