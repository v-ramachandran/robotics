#include<localization/GenericKalmanFilter.h>
using namespace Eigen;

template<int Dimensions>
Matrix<float, Dimensions, 1> GenericKalmanFilter<Dimensions>::apply(Eigen::Matrix<float, Dimensions, 1> newMeasurement) {
  measurement = calculateMeasurement(newMeasurement, measurement);
	measurement = newMeasurement;
	Matrix<float,Dimensions,1> tempState = A * state;
	Matrix<float,Dimensions,Dimensions> tempCovariance = A * covariance * A.transpose() + R;
	Matrix<float,Dimensions,Dimensions> temp = (C * tempCovariance * C.transpose()) + Q;
	const Matrix<float,Dimensions,Dimensions> tempInverse = temp.inverse();
	Matrix<float,Dimensions,Dimensions> kalmanGain = tempCovariance * C.transpose() * tempInverse;
	Matrix<float,Dimensions,1> newState = tempState + kalmanGain *(measurement - C*tempState);
	Matrix<float,Dimensions,Dimensions> newCovariance = tempCovariance - kalmanGain * C *tempCovariance;
	state = newState;
	covariance = newCovariance;
  return newState;
}

SimpleBallFilter::SimpleBallFilter() {
  A << 1,0,1,0,0.5f,0,
		   0,1,0,1,0,0.5f,
		   0,0,1,0,1,0,
		   0,0,0,1,0,1,
		   0,0,0,0,1,0,
		   0,0,0,0,0,1;
	C << 1,0,0,0,0,0,
	     0,1,0,0,0,0,
		   0,0,1,0,0,0,
		   0,0,0,1,0,0,
		   0,0,0,0,1,0,
		   0,0,0,0,0,1;
	R << 0.1,0,0,0,0,0,
		   0,0.1,0,0,0,0,
		   0,0,0.1,0,0,0,
		   0,0,0,0.1,0,0,
		   0,0,0,0,0.1,0,
		   0,0,0,0,0,0.1;
	Q << 0.1f,0,0,0,0,0,
		   0,0.1f,0,0,0,0,
		   0,0,0.1f,0,0,0,
		   0,0,0,0.1f,0,0,
		   0,0,0,0,0.1f,0,
		   0,0,0,0,0,0.1f;
	covariance << 1,0,0,0,0,0,
                0,1,0,0,0,0,
                0,0,1,0,0,0,
                0,0,0,1,0,0,
                0,0,0,0,1,0,
                0,0,0,0,0,1;
  state << 0,0,0,0,3.0f,-1.0f;
}

void SimpleBallFilter::reset() {
  A << 1,0,1,0,0.5f,0,
		   0,1,0,1,0,0.5f,
		   0,0,1,0,1,0,
		   0,0,0,1,0,1,
		   0,0,0,0,1,0,
		   0,0,0,0,0,1;
	C << 1,0,0,0,0,0,
	     0,1,0,0,0,0,
		   0,0,1,0,0,0,
		   0,0,0,1,0,0,
		   0,0,0,0,1,0,
		   0,0,0,0,0,1;
	R << 0.1,0,0,0,0,0,
		   0,0.1,0,0,0,0,
		   0,0,0.1,0,0,0,
		   0,0,0,0.1,0,0,
		   0,0,0,0,0.1,0,
		   0,0,0,0,0,0.1;
	Q << 0.1f,0,0,0,0,0,
		   0,0.1f,0,0,0,0,
		   0,0,0.1f,0,0,0,
		   0,0,0,0.1f,0,0,
		   0,0,0,0,0.1f,0,
		   0,0,0,0,0,0.1f;
	covariance << 1,0,0,0,0,0,
                0,1,0,0,0,0,
                0,0,1,0,0,0,
                0,0,0,1,0,0,
                0,0,0,0,1,0,
                0,0,0,0,0,1;
  state << 0,0,0,0,3.0f,-1.0f;
}

Matrix<float, 6, 1> SimpleBallFilter::apply(Eigen::Matrix<float, 6, 1> newMeasurement) {
  return GenericKalmanFilter<6>::apply(newMeasurement);
}

Matrix<float, 6, 1> SimpleBallFilter::calculateMeasurement(Matrix<float, 6, 1> newMeasurement, Matrix<float, 6, 1> measurement) {
  newMeasurement[2] = newMeasurement[0] - measurement[0];
	newMeasurement[3] = newMeasurement[1] - measurement[1];
	newMeasurement[4] = newMeasurement[2] - measurement[2];
	newMeasurement[5] = newMeasurement[3] - measurement[3];
  return newMeasurement;
}
