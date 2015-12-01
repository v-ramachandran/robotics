#ifndef BOUNDARY_SEGMENT_H
#define BOUNDARY_SEGMENT_H

#include <algorithm>

struct BoundarySegment {
	float averageDistance;
	float totalDistance;
	float maxDistance;
	int totalPoints;
	
	std::vector<LinePoint> linePoints;
	
	BoundarySegment() {
		averageDistance = 0.0;
		totalDistance = 0.0;
		maxDistance = 0.0;
		totalPoints = 0;
	}
	
	inline bool isPresent() {
		return totalPoints != 0;
	}
	
	inline void addLinePoint(LinePoint linePoint) {
		totalPoints = totalPoints + 1;
		totalDistance = linePoint.distance + totalDistance;
		maxDistance = max(linePoint.distance, maxDistance);
		averageDistance = totalDistance / totalPoints;
		
		linePoints.push_back(linePoint);
	}
}