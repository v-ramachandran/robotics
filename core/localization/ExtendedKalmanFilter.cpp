#include<localization/ExtendedKalmanFilter.h>
using namespace Eigen;
ExtendedBallFilter::ExtendedBallFilter(){
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

void ExtendedBallFilter::reset() {
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

Matrix<float,6,1> ExtendedBallFilter::updateFunction(Matrix<float,6,1> state){
  Matrix<float,6,1> tempState;
  tempState[0] = state[0] + state[2] + state[4]/2;
  tempState[1] = state[1] + state[3] + state[5]/2;
  tempState[2] = state[2] + state[4];
  tempState[3] = state[3] + state[5];
  tempState[4] = state[4];
  tempState[5] = state[5];
  return tempState;
}

Matrix<float,6,6> ExtendedBallFilter::findJacobiG(Matrix<float,6,1> state){
  Matrix<float,6,6> G;
  for(int j=0; j<6; ++j){
    Matrix<float,6,1> tempState1 = updateFunction(state);
    state[j] = state[j] + 0.2;
    Matrix<float,6,1> tempState2 = updateFunction(state);
    for(int i=0;i <6;++i){
      G(i,j) = (tempState2[i] - tempState1[i])/0.2;
    }
    state[j] = state[j] - 0.2;
}
return G;
}

Matrix<float,6,1> ExtendedBallFilter::extractionFunction(Matrix<float,6,1> state){
    Matrix<float,6,1> measurement;
    for(int i=0;i<6;++i){
      measurement[i] = state[i];

    }
    return measurement;
}

Matrix<float,6,6> ExtendedBallFilter::findJacobiH(Matrix<float,6,1> state){
  Matrix<float,6,6> G;
  for(int j=0; j<6; ++j){
    Matrix<float,6,1> tempState1 = extractionFunction(state);
    state[j] = state[j] + 0.2;
    Matrix<float,6,1> tempState2 = extractionFunction(state);
    for(int i=0;i <6;++i){
      G(i,j) = (tempState2[i] - tempState1[i])/0.2;
    }
    state[j] = state[j] - 0.2;
}
return G;
}

Matrix<float,6,1> ExtendedBallFilter::specificFunction(Matrix<float,6,1> newMeasurement){

	newMeasurement[2] = newMeasurement[0] - measurement[0];
	newMeasurement[3] = newMeasurement[1] - measurement[1];
	newMeasurement[4] = newMeasurement[2] - measurement[2];
	newMeasurement[5] = newMeasurement[3] - measurement[3];
  measurement = newMeasurement;
  Matrix<float,6,1> tempState = updateFunction(state);
  Matrix<float,6,6> G = findJacobiG(state);
  Matrix<float,6,6> H = findJacobiH(state);
  Matrix<float,6,6> tempCovariance = G * covariance * G.transpose() + R;
  Matrix<float,6,6> temp = (H * tempCovariance * H.transpose()) + Q;
	const Matrix<float,6,6> tempInverse = temp.inverse();
	Matrix<float,6,6> kalmanGain = tempCovariance * H.transpose() * tempInverse;
	Matrix<float,6,1> newState = tempState + kalmanGain *(measurement - extractionFunction(tempState));
	Matrix<float,6,6> newCovariance = tempCovariance - kalmanGain * H *tempCovariance;
	state = newState;
	covariance = newCovariance;
  return newState;
}
