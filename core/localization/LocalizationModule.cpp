#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger) {
  ballFilter = new ExtendedBallFilter();
  timesUnseen = 0;
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
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
  cache_.localization_mem->player = self.loc;
  printf("%d %d\n",self.loc.x, self.loc.y);
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  cache_.localization_mem->player = Point2D(-750,0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();
}

void LocalizationModule::processFrame() {
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Retrieve the robot's current location from localization memory
  // and store it back into world objects
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

    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window
    cache_.localization_mem->state[0] = ball.loc.x;
    cache_.localization_mem->state[1] = ball.loc.y;
    cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity() * 10000;
  } 
  //TODO: How do we handle not seeing the ball?
  else {
    timesUnseen = timesUnseen + 1;    
    if (timesUnseen > 10) {
      ball.distance = 10000.0f;
      ball.bearing = 0.0f;
      ballFilter->reset();
    }
  }
}
