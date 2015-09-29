#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>

using namespace Eigen;



BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

bool BeaconDetector::checkWhiteLowerPixels(ImageProcessor *processor, int leftX, int rightX, int depth) {
  int countWhite = 0;
  int width = rightX - leftX + 1;
  unsigned char * image = processor->getImg();
  unsigned char * color_table_ = processor->getColorTable();
  for(int y = depth+1; y <= depth+8; ++y){
    for(int x = leftX; x <= rightX; ++x){
      Color currentColor = ColorTableMethods::xy2color(image, color_table_, x, y, processor->getImageWidth());
      if(currentColor == c_WHITE) 
        countWhite++;
    }
  }
  float ratioWhite = (float)countWhite/(float)(width*5);
  return (ratioWhite > 0.5);   
}

bool BeaconDetector::checkColorUpperPixels(ImageProcessor *processor, int leftX, int rightX, int height) {
  int countBlue = 0;
  int countPink = 0;
  int countYellow = 0;
  int width = rightX - leftX + 1;
  unsigned char * image = processor->getImg();
  unsigned char * color_table_ = processor->getColorTable();
  for(int y = height - 8; y<height; ++y){
    for(int x = leftX; x <= rightX; ++x){
      Color currentColor = ColorTableMethods::xy2color(image, color_table_, x, y, processor->getImageWidth());
      if(currentColor == c_BLUE) 
        countBlue++;
      else if (currentColor == c_PINK)
        countPink++;
      else if (currentColor == c_YELLOW)
        countYellow++;
    }
  }
  float ratioBlue = (float)countBlue/(float)(width*5);
  float ratioPink = (float)countPink/(float)(width*5);
  float ratioYellow = (float)countYellow/(float)(width*5);
  return (ratioBlue > 0.5 || ratioPink > 0.5 || ratioYellow > 0.5); 
}

vector<int> BeaconDetector::findColoredBeacon(Color color1, Color color2, std::map<Color, struct DisjointSet> colorDisjointSets, ImageProcessor *processor) {

  float errorX, errorY;
  errorX = 15;
  errorY = 6;
  int minValue = 8;
  vector<int> box;
  
  for(std::set<TreeNode *>::iterator colorDisjointSet1 = colorDisjointSets[color1].rootSet.begin(); colorDisjointSet1 != colorDisjointSets[color1].rootSet.end(); ++colorDisjointSet1){
    if((!((*colorDisjointSet1)->hasMinimumWidth(minValue) && (*colorDisjointSet1)->hasMinimumHeight(minValue))) || checkColorUpperPixels(processor,(*colorDisjointSet1)->topleft->x,(*colorDisjointSet1)->bottomright->x, (*colorDisjointSet1)->topleft->y)){
      continue;
    }
    
    for(std::set<TreeNode *>::iterator colorDisjointSet2 = colorDisjointSets[color2].rootSet.begin(); colorDisjointSet2 != colorDisjointSets[color2].rootSet.end(); ++colorDisjointSet2){
  //    std::cout<<(*colorDisjointSet2)->topleft->x<<" "<<(*colorDisjointSet2)->topleft->y<<" "<<(*colorDisjointSet2)->bottomright->x<<" "<<(*colorDisjointSet2)->bottomright->y<<endl;
  //    std::cout<<"o "<<checkWhiteLowerPixels(processor, (*colorDisjointSet2)->topleft->x, (*colorDisjointSet2)->bottomright->x, (*colorDisjointSet2)->bottomright->y)<<endl;   
      if((*colorDisjointSet1)->hasSimilarWidth(*colorDisjointSet2,errorX) && (*colorDisjointSet1)->hasSimilarHeight(*colorDisjointSet2,errorY) && (*colorDisjointSet1)->isStackedAbove    (*colorDisjointSet2,errorX,errorY) && checkWhiteLowerPixels(processor, (*colorDisjointSet2)->topleft->x, (*colorDisjointSet2)->bottomright->x, (*colorDisjointSet2)->bottomright->y)) {
         box.push_back(std::min((*colorDisjointSet1)->topleft->x, (*colorDisjointSet2)->topleft->x));
         box.push_back((*colorDisjointSet1)->topleft->y);
         box.push_back(std::max((*colorDisjointSet1)->bottomright->x, (*colorDisjointSet2)->bottomright->x));
         box.push_back((*colorDisjointSet2)->bottomright->y);
         return box;
      }
    }
  }
  return box;
}
void BeaconDetector::findBeacons(std::map<Color, struct DisjointSet> colorDisjointSets, ImageProcessor * processor) {
  if(camera_ == Camera::BOTTOM) return;
  vector<int> boxYellowBlue =  findColoredBeacon(c_YELLOW, c_BLUE, colorDisjointSets, processor);
  vector<int> boxBlueYellow =  findColoredBeacon(c_BLUE, c_YELLOW, colorDisjointSets, processor);
  vector<int> boxYellowPink =  findColoredBeacon(c_YELLOW, c_PINK, colorDisjointSets, processor);
  vector<int> boxPinkYellow =  findColoredBeacon(c_PINK, c_YELLOW, colorDisjointSets, processor);
  vector<int> boxBluePink = findColoredBeacon(c_BLUE, c_PINK, colorDisjointSets, processor);
  vector<int> boxPinkBlue = findColoredBeacon(c_PINK, c_BLUE, colorDisjointSets, processor);

  map<WorldObjectType,int> heights = {
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_BLUE_YELLOW, 300 },
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_BLUE_PINK, 200 },
    { WO_BEACON_PINK_BLUE, 200 }
  };

  map<WorldObjectType,vector<int>> beacons = {
    { WO_BEACON_YELLOW_BLUE, boxYellowBlue },
    { WO_BEACON_BLUE_YELLOW, boxBlueYellow },
    { WO_BEACON_YELLOW_PINK, boxYellowPink },
    { WO_BEACON_PINK_YELLOW, boxPinkYellow },
    { WO_BEACON_BLUE_PINK, boxBluePink },
    { WO_BEACON_PINK_BLUE, boxPinkBlue },
  };

  for(auto beacon : beacons) {
    vector<int> box = beacon.second;
    if (box.empty()) continue;
    auto& object = vblocks_.world_object->objects_[beacon.first];
    object.imageCenterX = (box[0] + box[2]) / 2;
    object.imageCenterY = (box[1] + box[3]) / 2;
//    std::cout<< box[4] <<endl;
    auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[beacon.first]);
    object.visionDistance = cmatrix_.groundDistance(position);
    object.visionBearing = cmatrix_.bearing(position);
    object.seen = true;
    object.fromTopCamera = camera_ == Camera::TOP;
//    std::cout<<"distance! "<<getName(beacon.first)<<" "<<object.imageCenterX<<" "<< object.imageCenterY<<" "<< object.visionDistance<<endl;
    visionLog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(beacon.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }
}
