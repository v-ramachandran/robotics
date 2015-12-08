#include <vision/LineDetector.h>
#include <memory/TextLogger.h>
#include <memory/RobotStateBlock.h>
#include <memory/WorldObjectBlock.h>

using namespace Eigen;

LineDetector::LineDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void LineDetector::resetBoundarySegments(ImageProcessor* processor) {
  if (processor->isTopCamera()) {
    auto& woBoundarySegment = vblocks_.world_object->objects_[WO_TOP_BOUNDARY_SEGMENT];
    woBoundarySegment.fromTopCamera = true;
    woBoundarySegment.boundarySegment.reset();
    woBoundarySegment.seen = false;
  } else {
    auto& woBoundarySegment = vblocks_.world_object->objects_[WO_BOTTOM_BOUNDARY_SEGMENT];
    woBoundarySegment.fromTopCamera = false;
    woBoundarySegment.boundarySegment.reset();
    woBoundarySegment.seen = false;
  }
}

void LineDetector::findLinePointCandidates(ImageProcessor * processor){

  for (std::vector< LinePoint* >::iterator it = linePoints.begin() ; it != linePoints.end(); ++it) {
    delete (*it);
  } 
  linePoints.clear();

  unsigned char * image = processor->getImg();
  unsigned char * color_table_ = processor->getColorTable();
  int height = processor->getImageHeight();
  int width = processor->getImageWidth();
  int threshold = 20;
  int step = 5;
  
  resetBoundarySegments(processor);

  for (int y=0; y<height; ++y) {

    if (y % 5 == 0) {
      step = 1;
    } else {
      step = 5;
    }    
    
    int prevPosX, prevPosY, nextPosX, nextPosY, prevComparisonY, prevComparisonU, prevComparisonV, nextComparisonY, nextComparisonU, nextComparisonV;
    for (int x=0; x<width; x=x+step) {
      
      if ((x != 0) && (y % 5 == 0)) { 
        prevPosX = x-1;
        prevPosY = y;
        nextPosX = x+1;
        nextPosY = y;
      } else if ((y != 0) && (y % 5 != 0)) {
        prevPosX = x;
        prevPosY = y-1;
        nextPosX = x;
        nextPosY = y+1;
      } else {
        continue;
      }

      if ((x != width-1) && (y % 5 == 0)) { 
        nextPosX = x+1;
        nextPosY = y;
      } else if ((y != height-1) && (y % 5 != 0)) {
        nextPosX = x;
        nextPosY = y+1;
      } else {
        continue;
      }

      // current point being inspected
      int currentY, currentU, currentV;
      ColorTableMethods::xy2yuv(image, x, y, width, currentY, currentU, currentV);
      ColorTableMethods::xy2yuv(image, prevPosX, prevPosY, width, prevComparisonY, prevComparisonU, prevComparisonV);
      ColorTableMethods::xy2yuv(image, nextPosX, nextPosY, width, nextComparisonY, nextComparisonU, nextComparisonV);
      auto color = ColorTableMethods::yuv2color(color_table_, currentY, currentU, currentV);

      // Add point if passes threshold
      if ((abs(currentY - prevComparisonY) > threshold || abs(currentY - nextComparisonY) > threshold) && color == c_FIELD_GREEN) {
				
       	// Calculate the global position of the edge pixel
	      auto& self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
        auto p = cmatrix_.getWorldPosition(x,y, 1.0);
        float visionDistance = cmatrix_.groundDistance(p);
        float visionBearing = cmatrix_.bearing(p);
	      auto relativePosition = Point2D::getPointFromPolar(visionDistance, visionBearing); 
	      auto globalPosition = relativePosition.relativeToGlobal(self.loc, self.orientation);

        LinePoint* linePoint = new LinePoint();
        linePoint->PosX = x;
        linePoint->PosY = y;
        linePoint->isFalsePositive = true;
	      if (BoundaryLineMethods::isOnBoundary(globalPosition) ) { 
          LinePoint point; 
          point.y = currentY;
          point.u = currentU;
          point.v = currentV;
          point.globalPosX = globalPosition.x;
          point.globalPosY = globalPosition.y;
          point.relativePosX = relativePosition.x;
          point.relativePosY = relativePosition.y;
          point.PosX = x;
          point.PosY = y;
          point.distance = visionDistance;
          linePoint->isFalsePositive = false;

          if (processor->isTopCamera()) {
            auto& woBoundarySegment = vblocks_.world_object->objects_[WO_TOP_BOUNDARY_SEGMENT];
            woBoundarySegment.boundarySegment.addLinePoint(point);
            woBoundarySegment.seen = true;
          } else {
            auto& woBoundarySegment = vblocks_.world_object->objects_[WO_BOTTOM_BOUNDARY_SEGMENT];
            if(point.PosY < 200){
              woBoundarySegment.boundarySegment.addLinePoint(point);
              woBoundarySegment.seen = true;
            } else {
              linePoint->isFalsePositive = true;          
            }
          }

        }

//        if(processor->isTopCamera() || linePoint->PosY < 200){
        linePoints.push_back(linePoint);
//        }
      }
    }

/*    if (processor->isTopCamera()) {
    } else {
      auto& woBoundarySegment = vblocks_.world_object->objects_[WO_BOTTOM_BOUNDARY_SEGMENT];
      if (woBoundarySegment.seen) {
         std::cout << "BOTTOM: " << woBoundarySegment.boundarySegment.totalPoints << " " << woBoundarySegment.boundarySegment.maxDistance << " " << woBoundarySegment.boundarySegment.minDistance << " " << woBoundarySegment.boundarySegment.averageDistance << endl;
      } 
    } */
  }
}
