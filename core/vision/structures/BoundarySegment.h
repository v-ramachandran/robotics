#ifndef BOUNDARY_SEGMENT_H
#define BOUNDARY_SEGMENT_H

#include <algorithm>
#include <vision/structures/LinePoint.h>

struct BoundarySegment {
	float averageDistance;
	float totalDistance;
	float maxDistance;
  float minDistance;
	int totalPoints;
	
//	std::vector<LinePoint> linePoints;
	
	BoundarySegment() {
    minDistance = 1000000;
		averageDistance = 0.0;
		totalDistance = 0.0;
		maxDistance = 0.0;
		totalPoints = 0;
	}

  ~BoundarySegment() {
//    linePoints.clear();
    minDistance = 1000000;
		averageDistance = 0.0;
		totalDistance = 0.0;
		maxDistance = 0.0;
		totalPoints = 0;    
  }	

  bool isPresent();
  void reset();
  void addLinePoint(LinePoint linePoint);
};

#endif
