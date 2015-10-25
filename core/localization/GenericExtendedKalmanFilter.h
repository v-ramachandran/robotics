#ifndef CORE_LOCALIZATION_GEKALMANFILTER_H_
#define CORE_LOCALIZATION_GEKALMANFILTER_H_
#include<Eigen/Core>
#include<Eigen/LU>
using namespace Eigen;

template<int Dimensions>
class GenericExtendedKalmanFilter {
  public:
    virtual void reset() = 0;
    virtual Matrix<float, Dimensions, 1> apply(Eigen::Matrix<float, Dimensions, 1> newMeasurement);
    virtual Matrix<float, Dimensions, 1> calculateMeasurement(Matrix<float, Dimensions, 1> newMeasurement, Matrix<float, Dimensions, 1> measurement) = 0;
    virtual Matrix<float, Dimensions, 1> updateFunction(Matrix<float, Dimensions, 1> state) = 0;
    virtual Matrix<float, Dimensions, Dimensions> findJacobiG(Matrix<float,Dimensions ,1> state) = 0;
    virtual Matrix<float, Dimensions, 1> extractionFunction(Matrix<float, Dimensions, 1> state) = 0;
    virtual Matrix<float, Dimensions, Dimensions> findJacobiH(Matrix<float, Dimensions, 1> state) = 0;
    Matrix<float,Dimensions,1> state;
    Matrix<float,Dimensions,1> measurement;
    Matrix<float,Dimensions,Dimensions> covariance;
    Matrix<float,Dimensions,Dimensions> A, C, R, Q;
};

class SimpleExtendedBallFilter : public GenericExtendedKalmanFilter<6> {
  public:
	  SimpleExtendedBallFilter();
    void reset() override;  	
    Matrix<float, 6, 1> calculateMeasurement(Matrix<float, 6, 1> newMeasurement, Matrix<float, 6, 1> measurement) override;
    Matrix<float, 6, 1> apply(Eigen::Matrix<float, 6, 1> newMeasurement) override;
    Matrix<float, 6, 1> updateFunction(Matrix<float, 6, 1> state) override;
    Matrix<float, 6, 6> findJacobiG(Matrix<float,6 ,1> state) override;
    Matrix<float, 6, 1> extractionFunction(Matrix<float, 6, 1> state) override;
    Matrix<float, 6, 6> findJacobiH(Matrix<float, 6, 1> state) override;
};
#endif
