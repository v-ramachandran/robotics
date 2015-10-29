#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <localization/ParticleFilter.h>
#include <localization/Logging.h>

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger), pfilter_(new ParticleFilter(cache_, tlogger_)) {
  ballFilter = new ExtendedBallFilter();
  timesUnseen = 0;
}

LocalizationModule::~LocalizationModule() {
  delete pfilter_;
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("vision_joint_angles");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
  getOrAddMemoryBlock(cache_.joint,"vision_joint_angles");
}


// Load params that are defined in cfglocalization.py
void LocalizationModule::loadParams(LocalizationParams params) {
  params_ = params;
  printf("Loaded localization params for %s\n", params_.behavior.c_str());
}

// Perform startup initialization such as allocating memory
void LocalizationModule::initSpecificModule() {
  reInit();
}

// Initialize the localization module based on data from the LocalizationBlock
void LocalizationModule::initFromMemory() {
  reInit();
}

// Initialize the localization module based on data from the WorldObjectBlock
void LocalizationModule::initFromWorld() {
  reInit();
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  pfilter_->init(self.loc, self.orientation);
  cache_.localization_mem->player = self.loc;
  printf("%d %d\n",self.loc.x, self.loc.y);
}

void LocalizationModule::initWithFilterBeacons(bool isFirstField) {
  reInit();
  pfilter_->init(Point2D(750,0), 0.0f, isFirstField);
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  cache_.localization_mem->player = self.loc;
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  pfilter_->init(Point2D(750,0), 0.0f);
  cache_.localization_mem->player = Point2D(-750,0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();
}

void LocalizationModule::moveBall(const Point2D& position) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::movePlayer(const Point2D& position, float orientation) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::processFrame() {
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Process the current frame and retrieve our location/orientation estimate
  // from the particle filter
  pfilter_->processFrame();
//  self.loc = pfilter_->pose().translation;
//  self.orientation = pfilter_->pose().rotation;
  log(40, "Localization Update: x=%2.f, y=%2.f, theta=%2.2f", self.loc.x, self.loc.y, self.orientation * RAD_T_DEG);
  auto sloc = cache_.localization_mem->player;
  self.loc = sloc;    

  //TODO: modify this block to use your Kalman filter implementation
  if(ball.seen) {
    timesUnseen = 0;
    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

    // Update the ball in the WorldObject block so that it can be accessed in python
    Eigen::Matrix<float, 6, 1> measurement;
    measurement << globalBall.x, globalBall.y,0,0,0,0;
    auto ballState = ballFilter->specificFunction(measurement);
    auto estimatedGlobalBall = Point2D(ballState[0],ballState[1]);
    auto estimatedRelativeBall = estimatedGlobalBall.globalToRelative(self.loc, self.orientation);
   // ball.loc = globalBall;
    ball.loc.x = estimatedRelativeBall.x + ballState[2] * 3 + ballState[4] * 6.125;
    ball.loc.y = estimatedRelativeBall.y + ballState[3] * 3 + ballState[5] * 6.125;
    ball.endLoc = relBall;
   // ball.distance = ball.visionDistance;
    ball.distance = sqrt(ball.loc.x * ball.loc.x + ball.loc.y * ball.loc.y);
   // ball.bearing = ball.visionBearing;
    ball.bearing = atan(ball.loc.y/ball.loc.x);
    ball.absVel = Vector2D(ballState[2] + ballState[4] * 3, ballState[3] + ballState[5] * 3);
    // std::cout<<"velcoity --- "<<ball.absVel.x<<" "<<ball.absVel.y<<endl;
    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window
    cache_.localization_mem->state[0] = ball.loc.x;
    cache_.localization_mem->state[1] = ball.loc.y;
    cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity() * 10000;
  } 
  //TODO: How do we handle not seeing the ball?
  else {
    timesUnseen = timesUnseen + 1; 
    std::cout<<"NOT SEEN"<<endl;   
    if (timesUnseen > 10) {
      ball.distance = 10000.0f;
      ball.bearing = 0.0f;
      ballFilter->reset();
    }
  }
}
