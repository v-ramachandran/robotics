#include <vision/LineDetector.h>
#include <memory/TextLogger.h>

using namespace Eigen;

LineDetector::LineDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
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
	int threshold = 30;
  int step = 5;

  for (int y=0; y<height; ++y) {
    if (y % 5 == 0) {
      step = 1;
    } else {
      step = 5;
    }    
    
    int posX, posY, comparisonY, comparisonU, comparisonV;
    for (int x=0; x<width; x=x+step) {
      if ((x != 0) && (y % 5 == 0)) { 
        posX = x-1;
        posY = y;
      } else if ((y != 0) && (y % 5 != 0)) {
        posX = x;
        posY = y-1;
      } else {
        continue;
      }

      

      // current point being inspected
      int currentY, currentU, currentV;
		  ColorTableMethods::xy2yuv(image, x, y, width, currentY, currentU, currentV);
			ColorTableMethods::xy2yuv(image, posX, posY, width, comparisonY, comparisonU, comparisonV);

      // Add point if passes threshold
      if (abs(currentY - comparisonY) > threshold) {
	      LinePoint *point = new LinePoint();
        point->y = currentY;
        point->u = currentU;
        point->v = currentV;
	      point->PosX = x;
	      point->PosY = y;
        linePoints.push_back(point);
      }
    }
  }
}

