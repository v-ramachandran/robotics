#include <vision/LineDetector.h>
#include <memory/TextLogger.h>

//LineDetector::LineDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
//}

void LineDetector::findLinePointCandidates(ImageProcessor * processor){
 	unsigned char * image = processor->getImg();
  unsigned char * color_table_ = processor->getColorTable();
  int height = processor->getImageHeight();
  int width = processor->getImageWidth();
	
  for(int y=0; y<height; ++y){
    if(y%5==0){
      for(int x=0; x<width; ++x){
        Color currentColor = ColorTableMethods::xy2color(image, color_table_, x, y, width);
				if(currentColor==c_WHITE){
					LinePoint point;
					ColorTableMethods::xy2yuv(image, x, y, width, point.y, point.u, point.v);
					point.PosX = x;
					point.PosY = y;
					linePoints.push_back(point);
				}					 
      }
    }
    else{
      for(int x=0; x<width; x=x+5){
        Color currentColor = ColorTableMethods::xy2color(image, color_table_, x, y, processor->getImageWidth());
				if(currentColor==c_WHITE){
					if(currentColor==c_WHITE){
						LinePoint point;
						ColorTableMethods::xy2yuv(image, x, y, width, point.y, point.u, point.v);
						point.PosX = x;
						point.PosY = y;
						linePoints.push_back(point);
				}					  
      }
    }
  }

  
}
}

