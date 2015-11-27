#include <vision/LineDetector.h>
#include <memory/TextLogger.h>

//LineDetector::LineDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
//}

void LineDetector::findLinePointCandidates(ImageProcessor * processor){
 	unsigned char * image = processor->getImg();
  unsigned char * color_table_ = processor->getColorTable();
  int height = processor->getImageHeight();
  int width = processor->getImageWidth();
	int threshold =0;
  for(int y=0; y<height; ++y){
    if(y%5==0){
      for(int x=0; x<width; ++x){
				LinePoint point;
				ColorTableMethods::xy2yuv(image, x, y, width, point.y, point.u, point.v);
				point.PosX = x;
				point.PosY = y;
        Color currentColor = ColorTableMethods::xy2color(image, color_table_, x, y, width);
				if(y>threshold){
				
					linePoints.push_back(point);
				}					 
      }
    }
    else{
      for(int x=0; x<width; x=x+5){
				LinePoint point;
				ColorTableMethods::xy2yuv(image, x, y, width, point.y, point.u, point.v);
				point.PosX = x;
				point.PosY = y;
        Color currentColor = ColorTableMethods::xy2color(image, color_table_, x, y, processor->getImageWidth());
				if(y>threshold){
						
						linePoints.push_back(point);
				}
    	}
  	}

  
	}
}

