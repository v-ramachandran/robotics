#include<localization/KalmanFilter.h>
using namespace Eigen;
BallFilter::BallFilter(){
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
	Q << 0.1,0,0,0,0,0,
   		 0,0.1,0,0,0,0,
		   0,0,0.1,0,0,0,
		   0,0,0,0.1,0,0,
		   0,0,0,0,0.1,0,
		   0,0,0,0,0,0.1;
  covariance << 1,0,0,0,0,0,
                0,1,0,0,0,0,
                0,0,1,0,0,0,
                0,0,0,1,0,0,
                0,0,0,0,1,0,
                0,0,0,0,0,1;
	state << 0,0,0,0,3.0f,-1.0f;
}

void BallFilter::reset() {
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

Matrix<float,6,1> BallFilter::specificFunction(Matrix<float,6,1> newMeasurement){

	newMeasurement[2] = newMeasurement[0] - measurement[0];
	newMeasurement[3] = newMeasurement[1] - measurement[1];
	newMeasurement[4] = newMeasurement[2] - measurement[2];
	newMeasurement[5] = newMeasurement[3] - measurement[3];
	measurement = newMeasurement;
	Matrix<float,6,1> tempState = A * state;
	Matrix<float,6,6> tempCovariance = A * covariance * A.transpose() + R;
	Matrix<float,6,6> temp = (C * tempCovariance * C.transpose()) + Q;
	const Matrix<float,6,6> tempInverse = temp.inverse();
	Matrix<float,6,6> kalmanGain = tempCovariance * C.transpose() * tempInverse;
	Matrix<float,6,1> newState = tempState + kalmanGain *(measurement - C*tempState);
	Matrix<float,6,6> newCovariance = tempCovariance - kalmanGain * C *tempCovariance;
	state = newState;
	covariance = newCovariance;
  return newState;
}
