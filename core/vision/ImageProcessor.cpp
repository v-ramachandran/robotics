#include <vision/ImageProcessor.h>
#include <vision/BeaconDetector.h>
#include <vision/structures/TreeNode.h>
#include <iostream>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera), calibration_(NULL)
{
  enableCalibration_ = false;
  classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
  beacon_detector_ = new BeaconDetector(DETECTOR_PASS_ARGS);
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  classifier_->init(tl);
  beacon_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_, NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_, NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_, *abs_parts = vblocks_.body_model->abs_parts_;
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->img_top_;
  return vblocks_.image->img_bottom_;
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(RobotCalibration calibration){
  if(calibration_) delete calibration_;
  calibration_ = new RobotCalibration(calibration);
}

void ImageProcessor::processFrame(){
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  visionLog(30, "Process Frame camera %i", camera_);

  updateTransform();
  
  // Horizon calculation
  visionLog(30, "Calculating horizon line");
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 30000);
  vblocks_.robot_vision->horizon = horizon;
  visionLog(30, "Classifying Image", camera_);
  if(!classifier_->classifyImage(color_table_)) return;
  detectBall();
  getBallNodes();
  beacon_detector_->findBeacons();
}

void ImageProcessor::detectBall() {
}

void ImageProcessor::findBall(int& imageX, int& imageY) {
  imageX = imageY = 0;
}

int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

vector<struct TreeNode *> ImageProcessor::getBallNodes() {
  
  unsigned char* image = getImg();
  int height = getImageHeight();
  int width = getImageWidth();
  struct DisjointSet disjointSet;
  std::map<int,vector<struct TreeNode *>> rowNodeMap;

  for(int y=0; y<height; ++y){
    int rowStart = 0;
    int rowEnd = 0;
    std::vector<struct TreeNode *> treeNodes ;
    for(int x=0; x<width; ++x){
      if(ColorTableMethods::xy2color(image, color_table_, x, y, width)!=c_ORANGE) {
        if(rowStart != rowEnd){
          struct TreeNode * treeNode = disjointSet.makeset(y, rowStart, rowEnd, c_ORANGE);
          treeNodes.push_back(treeNode);
        }
        rowStart = x;
      }
      rowEnd = x;
    }
    rowNodeMap[y] = treeNodes;
  }

  for(int y=0; y<height-1; ++y) {
    for(std::vector<struct TreeNode *>::iterator currNodeItem = rowNodeMap[y].begin(); currNodeItem!= rowNodeMap[y].end(); ++currNodeItem) { 
      for(std::vector<struct TreeNode *>::iterator nextNodeItem = rowNodeMap[y+1].begin(); nextNodeItem!= rowNodeMap[y+1].end(); ++nextNodeItem) {
        if ((*currNodeItem)->hasOverlap(*nextNodeItem)) {
          if ((*nextNodeItem)->hasSelfAsParent()) {
            disjointSet.unionNodes(*currNodeItem, *nextNodeItem);
            disjointSet.find(*nextNodeItem);
          } else {
            disjointSet.mergeNodes(*currNodeItem, *nextNodeItem);
          }
          
        }
      }
    }
  }
  

 // for(std::set<struct TreeNode *>::iterator treeNode = disjointSet.rootSet.begin(); treeNode!= disjointSet.rootSet.end(); ++treeNode) {
   // std::cout<<(*treeNode)->topleft->x <<" " <<(*treeNode)->topleft->y<<" " << (*treeNode)->bottomright->x<<" " << (*treeNode)->bottomright->y<<" "<<endl;
 // } 

  std::vector<struct TreeNode *> ballNodes;
  
  for(std::set<struct TreeNode *>::iterator treeNode = disjointSet.rootSet.begin(); treeNode!= disjointSet.rootSet.end(); ++treeNode) {
    float width = (*treeNode)->bottomright->x - (*treeNode)->topleft->x;
    float height = (*treeNode)->bottomright->y - (*treeNode)->topleft->y;
    float ratio = abs((width-height)/(width+height));
    if((width >=3) && (height >=3) && (ratio<0.3)){
      ballNodes.push_back(*treeNode);
      std::cout<<width<<" "<<height<<endl;
    }  
  }

  
  return ballNodes;
}


std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  
  unsigned char* image = getImg();
  int height = getImageHeight();
  int width = getImageWidth();
  struct DisjointSet disjointSet;
  std::map<int,vector<struct TreeNode *>> rowNodeMap;

  for(int y=0; y<height; ++y){
    int rowStart = 0;
    int rowEnd = 0;
    std::vector<struct TreeNode *> treeNodes ;
    for(int x=0; x<width; ++x){
      if(ColorTableMethods::xy2color(image, color_table_, x, y, width)!=c_ORANGE) {
        if(rowStart != rowEnd){
          struct TreeNode * treeNode = disjointSet.makeset(y, rowStart, rowEnd, c_ORANGE);
          treeNodes.push_back(treeNode);
        }
        rowStart = x;
      }
      rowEnd = x;
    }
    rowNodeMap[y] = treeNodes;
  }

  for(int y=0; y<height-1; ++y) {
    for(std::vector<struct TreeNode *>::iterator currNodeItem = rowNodeMap[y].begin(); currNodeItem!= rowNodeMap[y].end(); ++currNodeItem) { 
      for(std::vector<struct TreeNode *>::iterator nextNodeItem = rowNodeMap[y+1].begin(); nextNodeItem!= rowNodeMap[y+1].end(); ++nextNodeItem) {
        if ((*currNodeItem)->hasOverlap(*nextNodeItem)) {
          if ((*nextNodeItem)->hasSelfAsParent()) {
            disjointSet.unionNodes(*currNodeItem, *nextNodeItem);
            disjointSet.find(*nextNodeItem);
          } else {
            disjointSet.mergeNodes(*currNodeItem, *nextNodeItem);
          }
          std::cout<<disjointSet.rootSet.size()<<endl;  
        }
      }
    }
  }
  
  std::cout<<disjointSet.rootSet.empty()<<endl;  
  
  for(std::set<struct TreeNode *>::iterator treeNode = disjointSet.rootSet.begin(); treeNode!= disjointSet.rootSet.end(); ++treeNode) {
    std::cout<<(*treeNode)->topleft->x <<" " <<(*treeNode)->topleft->y<<" " << (*treeNode)->bottomright->x<<" " << (*treeNode)->bottomright->y<<" "<<endl;
  } 

  
  return std::vector<BallCandidate*>();
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  return NULL;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->loaded_;
}
