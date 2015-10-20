#include <vision/ImageProcessor.h>
#include <vision/BeaconDetector.h>
#include <stdexcept>
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
  getBlobNodes();
  detectBall();
  detectGoal();
 beacon_detector_->findBeacons(colorDisjointSets, this);
}

void ImageProcessor::detectBall() {

  BallCandidate* ballCandidate = getBestBallCandidate();
  if(!ballCandidate) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];


  int imageX = ballCandidate->centerX;
  int imageY = ballCandidate->centerY;
  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;
  ball->radius = ballCandidate->radius;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  ball->seen = true;
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

void ImageProcessor::getBlobNodes() {
  
  unsigned char* image = getImg();
  int height = getImageHeight();
  int width = getImageWidth();
  std::map<Color,std::map<int,vector<struct TreeNode *>>> colorRowNodeMap;
  for(std::map<Color, struct DisjointSet>::iterator disjointSet = colorDisjointSets.begin(); disjointSet != colorDisjointSets.end(); ++disjointSet){
    disjointSet->second.clear();
  }
  
  for(int y=0; y<height; ++y){
    int rowStart = 0;
    int rowEnd = 0;
    Color currentColor = ColorTableMethods::xy2color(image, color_table_, rowStart, rowEnd, width);
    std::vector<struct TreeNode *> treeNodes ;
    for(int x=0; x<width; ++x) {
      Color detectedColor = ColorTableMethods::xy2color(image, color_table_, x, y, width);
      if (currentColor != detectedColor) {
        // save old run
        if ((camera_ == Camera::TOP && (!(currentColor == c_FIELD_GREEN || currentColor == c_UNDEFINED || currentColor == c_ROBOT_WHITE || currentColor == c_WHITE))) || 
            (camera_ == Camera::BOTTOM && (currentColor == c_ORANGE))) {
          struct TreeNode* treeNode = colorDisjointSets[currentColor].makeset(y, rowStart, rowEnd, currentColor);
          colorRowNodeMap[currentColor][y].push_back(treeNode);
        }
        // start new run
        rowStart = x;
      } 
      rowEnd = x;
      currentColor = detectedColor;
    }
  }

  for(std::map<Color, struct DisjointSet>::iterator disjointSet = colorDisjointSets.begin(); disjointSet != colorDisjointSets.end(); ++disjointSet){
    Color color = disjointSet->first; 
    for(int y=0; y<height-1; ++y) {
      for(std::vector<struct TreeNode *>::iterator currNodeItem = colorRowNodeMap[color][y].begin(); currNodeItem!= colorRowNodeMap[color][y].end(); ++currNodeItem) { 
        for(std::vector<struct TreeNode *>::iterator nextNodeItem = colorRowNodeMap[color][y+1].begin(); nextNodeItem!= colorRowNodeMap[color][y+1].end(); ++nextNodeItem) {
          if ((*currNodeItem)->hasOverlap(*nextNodeItem)) {
            if ((*nextNodeItem)->hasSelfAsParent()) {
              disjointSet->second.unionNodes(*currNodeItem, *nextNodeItem);
              disjointSet->second.find(*nextNodeItem);
            } else {
              disjointSet->second.mergeNodes(*currNodeItem, *nextNodeItem);
            }
            
          }
        }
      }
    }
  
  }
   
   for(std::map<Color, struct DisjointSet>::iterator disjointSet = colorDisjointSets.begin(); disjointSet != colorDisjointSets.end(); ++disjointSet){
     Color color = disjointSet->first;
     struct DisjointSet colorDisjointSet = disjointSet->second;
     for(int y=0; y<height-1; ++y) {
       for(std::vector<struct TreeNode *>::iterator currNodeItem = colorRowNodeMap[color][y].begin(); currNodeItem!= colorRowNodeMap[color][y].end(); ++currNodeItem) { 
         int height = (*currNodeItem)->bottomright->y - (*currNodeItem)->topleft->y;
         int width = (*currNodeItem)->bottomright->x - (*currNodeItem)->topleft->x;
         if(colorDisjointSet.rootSet.find(*currNodeItem) == colorDisjointSet.rootSet.end()){
           
           delete (*currNodeItem); 
         } else if (height <= 3 || width <= 3) {
           colorDisjointSet.rootSet.erase(*currNodeItem);
           //delete (*currNodeItem);
         }
       }
     }
   }
}

bool ImageProcessor::tiltAngleTest(struct TreeNode * treenode, float threshold){
  float imgCenterY = 120.0;
  float BBoxCentroidY = (treenode->bottomright->y - treenode->topleft->y)/2;
  float FocalPixConstant = 120;
  float RoboCamTilt = 0;
  float tilt = atan((imgCenterY - BBoxCentroidY)/FocalPixConstant) + RoboCamTilt;
  return (tilt <= threshold);
}

bool ImageProcessor::isSquare(struct TreeNode * treeNode){
  float width = (treeNode)->bottomright->x - (treeNode)->topleft->x;
  float height = (treeNode)->bottomright->y - (treeNode)->topleft->y;
  float ratio = abs((width-height)/(width+height));
  return (ratio<=0.3);    
}

bool ImageProcessor::isAtleastMinimumSize(struct TreeNode * treeNode){
  float width = (treeNode)->bottomright->x - (treeNode)->topleft->x;
  float height = (treeNode)->bottomright->y - (treeNode)->topleft->y;
  return ((width>=5)&&(height>=5));
}

bool ImageProcessor::hasBallAspectRatio(struct TreeNode * treeNode) {
  float width = (treeNode)->bottomright->x - (treeNode)->topleft->x;
  float height = (treeNode)->bottomright->y - (treeNode)->topleft->y;
  return (abs((width/height)-1) < 0.3);
}

bool ImageProcessor::hasMinimumArea(struct TreeNode * treeNode){
  return (treeNode->numberOfPixels > 24);
}

bool ImageProcessor::isCircularArea(struct TreeNode * treeNode){
  
  float width = (treeNode)->bottomright->x - (treeNode)->topleft->x;
  float height = (treeNode)->bottomright->y - (treeNode)->topleft->y;
  float expectedArea = width * height;
  return (treeNode->numberOfPixels / expectedArea) > .7;
}
std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  
  std::vector<struct BallCandidate*> ballNodes;
  struct DisjointSet disjointSet = colorDisjointSets[c_ORANGE];
  for(std::set<struct TreeNode *>::iterator treeNode = disjointSet.rootSet.begin(); treeNode!= disjointSet.rootSet.end(); ++treeNode) {
    float width = (*treeNode)->bottomright->x - (*treeNode)->topleft->x;
    float height = (*treeNode)->bottomright->y - (*treeNode)->topleft->y;
    float ratio = abs((width-height)/(width+height));
    if(isSquare(*treeNode) && (isAtleastMinimumSize(*treeNode)) && isCircularArea(*treeNode) && hasMinimumArea(*treeNode) && hasBallAspectRatio(*treeNode)){
      struct BallCandidate* ball = new BallCandidate();
      ball->width = width;
      ball->height = height;
      ball->centerX = (*treeNode)->topleft->x + (width/2);
      ball->centerY = (*treeNode)->topleft->y + (height/2);
      ball->radius = (width > height) ? (width/2) : (height/2);
      ballNodes.push_back(ball);
    }  
  }
  
  return ballNodes;
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  
  // TODO(ankitade): choose most appropriate one instead of best.  
  std::vector<struct BallCandidate*> balls = getBallCandidates();
  float margin = 100000.0;
  BallCandidate* candidate = NULL;
  for(std::vector<struct BallCandidate*>::iterator ball = balls.begin(); ball != balls.end(); ++ball) {
    // Position p = cmatrix_.getWorldPosition((*ball)->centerX, (*ball)->centerY, (*ball)->height); 
    Position q = cmatrix_.getWorldPosition((*ball)->centerX, (*ball)->centerY, 32.5);
    
    if(camera_ == Camera::TOP) {
      float h = cmatrix_.groundDistance(q);
//      float g = cmatrix_.groundDistance(p);
//      if (abs(((*ball)->width / 2) - (cmatrix_.getCameraWidthByDistance(g, 65)/2)) < 2 && abs(((*ball)->height / 2) - (cmatrix_.getCameraHeightByDistance(g, 65)/2)) < 2) {
//        return *ball;
//      } 
      float calculatedCameraWidth = cmatrix_.getCameraWidthByDistance(h, 65);
      float calculatedCameraHeight = cmatrix_.getCameraHeightByDistance(h, 65);
      float expectedValue = (*ball)->width > (*ball)->height ? calculatedCameraWidth : calculatedCameraHeight;
      float actualValue = (*ball)->width > (*ball)->height ? (*ball)->width : (*ball)->height;
    //  std::cout << "Values " << expectedValue << " " << actualValue << endl;
      float currentMargin = abs(expectedValue - actualValue);      
      if (currentMargin < 4 && currentMargin < margin) {        
        candidate = *ball;
        margin = currentMargin;
      }
    } else {
    //  std::cout << (*ball)-> width << endl;      
      if (((*ball)->width < 65) && ((*ball)->width > 30) && ((*ball)->height < 65) && ((*ball)->height > 30)) {
        return *ball;
      } 
    }
  }
  return candidate;
}

bool ImageProcessor::goalAspectRatioTest(struct TreeNode * node){

  float height = node->bottomright->y - node->topleft->y;
  float width = node->bottomright->x - node->topleft->x;  
  float ratio = width/height; 
  return (ratio>=1.1 && ratio<=3.0);
}

void ImageProcessor::detectGoal() {
  TreeNode* goalCandidate = getBestGoalCandidate();
  if(!goalCandidate) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OWN_GOAL];

  float width = goalCandidate->bottomright->x - goalCandidate->topleft->x;
  float height = goalCandidate->bottomright->y - goalCandidate->topleft->y;
  int imageX = goalCandidate->topleft->x + (width/2);
  int imageY = goalCandidate->topleft->y + (height/2);
  goal->imageCenterX = imageX;
  goal->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->visionDistance = cmatrix_.groundDistance(p);

  goal->seen = true;
}

std::vector<TreeNode*> ImageProcessor::getGoalCandidates() {
  
  std::vector<struct TreeNode*> goalNodes;
  struct DisjointSet disjointSet = colorDisjointSets[c_BLUE];
  for(std::set<struct TreeNode *>::iterator treeNode = disjointSet.rootSet.begin(); treeNode!= disjointSet.rootSet.end(); ++treeNode) {
    float width = (*treeNode)->bottomright->x - (*treeNode)->topleft->x;
    float height = (*treeNode)->bottomright->y - (*treeNode)->topleft->y;
    float area = width * height;
    float numberPixels = (*treeNode)->numberOfPixels;

    if((width>=5) && (height>=5) && ((numberPixels/area) > .6) && (numberPixels > 1100)){
      goalNodes.push_back(*treeNode);
    }  
  }
  
  return goalNodes;
}

struct TreeNode* ImageProcessor::getBestGoalCandidate(){
    
  std::vector<struct TreeNode *> goals = getGoalCandidates();
  for(std::vector<struct TreeNode *>::iterator goal = goals.begin(); goal != goals.end(); ++goal){
    float width = (*goal)->bottomright->x - (*goal)->topleft->x;
    float height = (*goal)->bottomright->y - (*goal)->topleft->y;
    float centerX = (*goal)->topleft->x + (width/2);
    float centerY = (*goal)->topleft->y + (height/2);
    Position p = cmatrix_.getWorldPosition(centerX, centerY, height);
    float g = cmatrix_.groundDistance(p);
  
    if (goalAspectRatioTest(*goal)) {
      return *goal;
    }
  }
  return NULL;
}


void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->loaded_;
}
