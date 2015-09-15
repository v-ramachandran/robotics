#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>

using namespace Eigen;



BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}



vector<int> BeaconDetector::findColoredBeacon(Color color1, Color color2, std::map<Color, struct DisjointSet> colorDisjointSets){

  float errorX, errorY;
  errorX = errorY = 8;
  int minValue = 7;
  vector<int> box;
 // for(std::set<TreeNode *>::iterator colorDisjointSet1 = colorDisjointSets[color1].rootSet.begin(); colorDisjointSet1 != colorDisjointSets[color1].rootSet.end(); ++colorDisjointSet1){
  //  std::cout<<color1<<" "<< (*colorDisjointSet1)->topleft->x<<" "<<(*colorDisjointSet1)->topleft->y<<" "<<(*colorDisjointSet1)->bottomright->x<<" "<<(*colorDisjointSet1)->bottomright->y<<endl;

 // }
  
  for(std::set<TreeNode *>::iterator colorDisjointSet1 = colorDisjointSets[color1].rootSet.begin(); colorDisjointSet1 != colorDisjointSets[color1].rootSet.end(); ++colorDisjointSet1){
    if(!((*colorDisjointSet1)->hasMinimumWidth(minValue) && (*colorDisjointSet1)->hasMinimumHeight(minValue))){
      continue;
    }
    // std::cout<<color1<<" "<< (*colorDisjointSet1)->topleft->x<<" "<<(*colorDisjointSet1)->topleft->y<<" "<<(*colorDisjointSet1)->bottomright->x<<" "<<(*colorDisjointSet1)->bottomright->y<<endl;
    for(std::set<TreeNode *>::iterator colorDisjointSet2 = colorDisjointSets[color2].rootSet.begin(); colorDisjointSet2 != colorDisjointSets[color2].rootSet.end(); ++colorDisjointSet2){
//      std::cout<<color2<<" "<< (*colorDisjointSet2)->topleft->x<<" "<<(*colorDisjointSet2)->topleft->y<<" "<<(*colorDisjointSet2)->bottomright->x<<" "<<(*colorDisjointSet2)->bottomright->y<<endl;
      if((*colorDisjointSet1)->hasSimilarWidth(*colorDisjointSet2,errorX) && (*colorDisjointSet1)->hasSimilarHeight(*colorDisjointSet2,errorY) && (*colorDisjointSet1)->isStackedAbove(*colorDisjointSet2,errorX,errorY) ){// &&(*colorDisjointSet1)->hasSimilarDensity(*colorDisjointSet2)){
        for(std::set<TreeNode *>::iterator whiteDisjointSet = colorDisjointSets[c_WHITE].rootSet.begin(); whiteDisjointSet != colorDisjointSets[c_WHITE].rootSet.end(); ++whiteDisjointSet ){
          if ((color1 == c_YELLOW && color2 == c_BLUE) || (color1 == c_BLUE && color2 == c_YELLOW)) {
            int heightColor1 = (*colorDisjointSet1)->bottomright->y - (*colorDisjointSet1)->topleft->y;
            int heightColor2 = (*colorDisjointSet2)->bottomright->y - (*colorDisjointSet2)->topleft->y;
            if((*whiteDisjointSet)->hasMinimumWidth(minValue) && (*whiteDisjointSet)->hasMinimumHeight(minValue)){
//            std::cout<<"white"<<" "<< (*whiteDisjointSet)->topleft->x<<" "<<(*whiteDisjointSet)->topleft->y<<" "<<(*whiteDisjointSet)->bottomright->x<<" "<<(*whiteDisjointSet)->bottomright->y<<endl;
//            std::cout<<"cand"<<" "<< (*whiteDisjointSet)->hasSimilarWidth(*colorDisjointSet2,errorX) <<" "<<(*whiteDisjointSet)->hasExpectedHeight(heightColor1 + heightColor2,errorY)<<" "<<(*colorDisjointSet2)->isStackedAbove(*whiteDisjointSet,errorX,errorY)<<endl;

            }

            if(((*whiteDisjointSet)->hasSimilarWidth(*colorDisjointSet2,errorX) && (*whiteDisjointSet)->hasExpectedHeight(heightColor1 + heightColor2,errorY*2) && (*colorDisjointSet2)->isStackedAbove(*whiteDisjointSet,errorX,errorY)) || (*colorDisjointSet2)->isContainedWithin(*whiteDisjointSet, errorX)){
               box.push_back(std::min((*colorDisjointSet1)->topleft->x, (*colorDisjointSet2)->topleft->x));
               box.push_back((*colorDisjointSet1)->topleft->y);
               box.push_back(std::max((*colorDisjointSet1)->bottomright->x, (*colorDisjointSet2)->bottomright->x));
               box.push_back((*colorDisjointSet2)->bottomright->y);
               int heightWhite = (*colorDisjointSet2)->isStackedAbove(*whiteDisjointSet,errorX,errorY) ? ((*whiteDisjointSet)->bottomright->y - (*whiteDisjointSet)->topleft->y): (heightColor1 + heightColor2);
               float scale = 2;          
              box.push_back(scale*(heightColor1 + heightColor2 + heightWhite));
              return box;
            }
          } else {
            if((*colorDisjointSet2)->hasSimilarWidth(*whiteDisjointSet,errorX) && (*colorDisjointSet2)->hasSimilarHeight(*whiteDisjointSet,errorY) && (*colorDisjointSet2)->isStackedAbove(*whiteDisjointSet,errorX,errorY)){
               box.push_back(std::min((*colorDisjointSet1)->topleft->x, (*colorDisjointSet2)->topleft->x));
               box.push_back((*colorDisjointSet1)->topleft->y);
               box.push_back(std::max((*colorDisjointSet1)->bottomright->x, (*colorDisjointSet2)->bottomright->x));
               box.push_back((*colorDisjointSet2)->bottomright->y);
               int heightColor1 = (*colorDisjointSet1)->bottomright->y - (*colorDisjointSet1)->topleft->y;
               int heightColor2 = (*colorDisjointSet2)->bottomright->y - (*colorDisjointSet2)->topleft->y;
               int heightWhite = (heightColor1 + heightColor2)/2;
               float scale = 2.45;          
              box.push_back(scale*(heightColor1 + heightColor2 + heightWhite));
              return box;
            }
          }
        }
      }
    }
  }

  return box;
}
void BeaconDetector::findBeacons(std::map<Color, struct DisjointSet> colorDisjointSets) {
  if(camera_ == Camera::BOTTOM) return;
  vector<int> boxYellowBlue =  findColoredBeacon(c_YELLOW, c_BLUE, colorDisjointSets);
  vector<int> boxBlueYellow =  findColoredBeacon(c_BLUE, c_YELLOW, colorDisjointSets);
  vector<int> boxYellowPink =  findColoredBeacon(c_YELLOW, c_PINK, colorDisjointSets);
  vector<int> boxPinkYellow =  findColoredBeacon(c_PINK, c_YELLOW, colorDisjointSets);
  vector<int> boxBluePink = findColoredBeacon(c_BLUE, c_PINK, colorDisjointSets);
  vector<int> boxPinkBlue = findColoredBeacon(c_PINK, c_BLUE, colorDisjointSets);

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
    std::cout<<getName(beacon.first)<<" "<<object.imageCenterX<<" "<< object.imageCenterY<<" "<< object.visionDistance<<endl;
    visionLog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(beacon.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }
}
