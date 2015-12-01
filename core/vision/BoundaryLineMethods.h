#ifndef BOUNDARY_LINE_METHODS_H
#define BOUNDARY_LINE_METHODS_H

#include <common/UTField.h>
#include <cmath>
#include <math/Pose2D.h>

class ColorTableMethods {
  public:
		static const int ERROR = 100;
    static inline bool isOnBoundary(Point2D position) {
      return (isEqualWithError(HALF_FIELD_X, position.x, ERROR) || isEqualWithError(NEG_HALF_FIELD_X, position.x, ERROR)) && 
				(isEqualWithError(HALF_FIELD_Y, position.y, ERROR) || isEqualWithError(NEG_HALF_FIELD_Y, position.y, ERROR));
		}

	private:
		static inline bool isEqualWithError(int target, int value, int error) {
			return (abs(target - value) <= error);
		}
};

#endif
