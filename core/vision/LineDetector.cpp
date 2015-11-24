#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>

LineDetector::LineDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void findLinePointCandidates(){
  unsigned char* image = getImg();
  int height = getImageHeight();
  int width = getImageWidth();
/*  for(int y=0; y<height; ++y){
    if(y%5==0){
      for(int x=0; x<width; ++x){
        
      }
    }
    else{
      for(int x=0; x<width; x=x+5){
        
      }
    }
  }*/

  
}
void LineDetector::findLine(){
 
}
