#ifndef BOUNDARY_LINE_METHODS_H
#define BOUNDARY_LINE_METHODS_H

#include <common/UTField.h>
#include <cmath>
#include <math/Pose2D.h>

class BoundaryLineMethods {
  public:
    static const int ERROR = 300;
		static const int ERROR_OUT = 500;
    static const int ERROR_IN = 200;
		static const int ERROR_EXACT = 200;
    static inline bool isOnLeftBoundary(Point2D position){
      bool condition1 = abs(position.x) < HALF_FIELD_X + ERROR_EXACT;
      bool condition2 = (position.y < (HALF_FIELD_Y + ERROR_OUT)) && (position.y > (HALF_FIELD_Y - ERROR_IN));
      return condition1 && condition2;
  //    return (isEqualWithError(HALF_FIELD_Y, position.y, ERROR) && (abs(position.x) < HALF_FIELD_X + ERROR_EXACT));
    }
    static inline bool isOnRightBoundary(Point2D position){
      bool condition1 = abs(position.x) < HALF_FIELD_X + ERROR_EXACT;
      bool condition2 = (position.y > (NEG_HALF_FIELD_Y - ERROR_OUT)) && (position.y < (NEG_HALF_FIELD_Y + ERROR_IN));
      return condition1 && condition2;
 //     return (isEqualWithError(NEG_HALF_FIELD_Y, position.y, ERROR) && (abs(position.x) < HALF_FIELD_X + ERROR_EXACT));
    }
    static inline bool isOnTopBoundary(Point2D position){
      bool condition1 = abs(position.y) < HALF_FIELD_Y + ERROR_EXACT;
      bool condition2 = (position.x < (HALF_FIELD_X + ERROR_OUT)) && (position.x > (HALF_FIELD_X - ERROR_IN));
      return condition1 && condition2;
 //     return (isEqualWithError(HALF_FIELD_X, position.x, ERROR) && (abs(position.y) < HALF_FIELD_Y + ERROR_EXACT));
    }
    static inline bool isOnBottomBoundary(Point2D position){
      bool condition1 = abs(position.y) < HALF_FIELD_Y + ERROR_EXACT;
      bool condition2 = (position.x > (NEG_HALF_FIELD_X - ERROR_OUT)) && (position.x < (NEG_HALF_FIELD_X + ERROR_IN));
      return condition1 && condition2;
 //     return (isEqualWithError(NEG_HALF_FIELD_X, position.x, ERROR) && (abs(position.y) < HALF_FIELD_Y + ERROR_EXACT));
    }

    static inline bool isOnBoundary(Point2D position) {
      return isOnLeftBoundary(position) || isOnRightBoundary(position) || isOnTopBoundary(position) || isOnBottomBoundary(position);
		}

    static inline bool isOnCurve(Point2D position){
      int radius = 600;
      int centerX = NEG_HALF_FIELD_X;
      int centerY = 0;
      int distance = sqrt((position.x - centerX)*(position.x - centerX) + (position.y - centerY)*(position.y - centerY));
      return isEqualWithError(radius, distance, ERROR);
    }

	private:
		static inline bool isEqualWithError(int target, int value, int error) {
			return (abs(target - value) <= error);
		}
};

#endif
