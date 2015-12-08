#ifndef LINEPOINT_H
#define LINEPOINT_H

/// @ingroup vision
struct LinePoint {
  unsigned short Width;
  float PosX, PosY; 
  float globalPosX, globalPosY, relativePosX, relativePosY;
  float distance;
  int y, u, v;
  bool isFalsePositive;
};

#endif
