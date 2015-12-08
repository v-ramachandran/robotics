#include <vision/structures/BoundarySegment.h>
#include <algorithm>

bool BoundarySegment::isPresent() {
	return totalPoints != 0;
}
	
void BoundarySegment::addLinePoint(LinePoint linePoint) {
	totalPoints = totalPoints + 1;
	totalDistance = linePoint.distance + totalDistance;
	maxDistance = std::max(linePoint.distance, maxDistance);
  minDistance = std::min(linePoint.distance, minDistance);
	averageDistance = totalDistance / totalPoints;
// 	linePoints.push_back(linePoint);
}

void BoundarySegment::reset() {
	averageDistance = 0.0;
	totalDistance = 0.0;
	maxDistance = 0.0;
	totalPoints = 0;    
  minDistance = 1000000;  
}
