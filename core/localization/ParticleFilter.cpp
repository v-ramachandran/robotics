#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/WorldObjectBlock.h>



ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::init(Point2D loc, float orientation) {

  particleIndices.resize(numParticles);
  std::iota(particleIndices.begin(), particleIndices.end(), 0);

  mean_.translation = loc;
  mean_.rotation = orientation;
  particles().resize(numParticles);
  int x = xMin;
  int y = yMin;
  for(auto& p : particles()) {
 //   p.x = x;
 //   p.y = y;
    p.x = rand_.sampleN(0, 750);
    p.y = rand_.sampleN(0, 350);
    p.t = rand_.sampleN(0, M_PI / 2);
    p.w = rand_.sampleU();
 //   y++;
 //   if (y > yMax) {
 //     x++;
 //     y = yMin;
 //   }
  }
}

bool ParticleFilter::isEqual(float x , float y){
  float epsilon = 0.00001;
  return std::abs(x-y) <= epsilon;
}

void ParticleFilter::propagateToNext() {
	
  const auto& disp = cache_.odometry->displacement;
	log(41, "Updating particles from odometry: %2.f,%2.f @ %2.8f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  printf("Updating particles from odometry: %2.f,%2.f @ %2.8f \n", disp.translation.x, disp.translation.y, disp.rotation);
	for(auto& p : particles()) { 
		float meanT = disp.rotation + p.t;
		p.t = disp.rotation + p.t;
//    if (!isEqual(disp.rotation, 0.0)) {
       p.t = p.t + rand_.sampleN(0, 0.1);
//    }
		float meanX = disp.translation.x + p.x;
		p.x = disp.translation.x * cos(p.t) + disp.translation.y * sin(p.t) + p.x;
		float meanY = disp.translation.y  + p.y;
		p.y = disp.translation.x * sin(p.t)+ disp.translation.y * cos(p.t) + p.y;

//    if(!isEqual(disp.translation.x,0) || !isEqual(disp.translation.y,0)){
      p.x += rand_.sampleN(0, 2);
      p.y += rand_.sampleN(0, 2);
//    }
    p.w = p.w;
    
	}
 
}

float ParticleFilter::gaussianProbability(float x, float mean, float var){
	return exp(-1 *((x - mean) * (x - mean))/ (2 * var));
}

bool ParticleFilter::checkBeaconVisibility(Point2D beaconLoc, Particle p ){
	float angle;
  angle = Point2D(p.x,p.y).getBearingTo(beaconLoc, p.t);
	return ( angle <= M_PI/6 && angle >= -1 * M_PI/6); 	
}

float ParticleFilter::createParticleWeights() {
  int beaconColors[6] = {WO_BEACON_YELLOW_PINK, WO_BEACON_PINK_YELLOW, WO_BEACON_PINK_BLUE, WO_BEACON_BLUE_PINK, WO_BEACON_YELLOW_BLUE, WO_BEACON_BLUE_YELLOW };
  int maxPenalty = 70;
  int maxPenaltyPadding = 20;
  int totalWeight = 0;
  for (auto color : beaconColors) {
    
  }
  for(auto& p : particles()) {
    p.w = 600;
    for (int i = 0; i < 6; ++i) {
      auto &object = cache_.world_object->objects_[beaconColors[i]];
      if (object.seen && checkBeaconVisibility(object.loc, p)) {
        float meanDistance = sqrt((object.loc.x - p.x) * (object.loc.x - p.x) + (object.loc.y - p.y) * (object.loc.y - p.y));
        float measurementDistance = object.visionDistance;
        cout << "mean measurement distance " << meanDistance << " " << measurementDistance << endl;
        float probability = gaussianProbability(measurementDistance, meanDistance, 20000);
        float penalty = min((maxPenalty - maxPenaltyPadding),(1-probability)*100); // p=0 -> 50, p=1 -> 0
        p.w = p.w - penalty;
        cout << i << "|" << p.x << "|" << object.loc.x << "|" << p.y << "|" << object.loc.y << "|" << object.loc.x << "|" << object.loc.y << "|" << probability << "|" << penalty << "|" << p.w << endl;
      } else if (!(object.seen) && !(checkBeaconVisibility(object.loc, p))) {
        p.w = p.w - min(50, abs((rand_.sampleN(0,1))*35));
      } else {
        p.w = 150;
        break;
      } 
    } 
  }

  return 0.0;
}
/*
float ParticleFilter::createParticleWeights() {

	// Generate random particles for demonstration
  Point2D beacons[6] = {Point2D(-1500,-1000), Point2D(-1500,1000), Point2D(0,-1000), Point2D(0,1000), Point2D(1500,-1000), Point2D(1500,1000)};
  int beaconColors[6] = {WO_BEACON_YELLOW_PINK, WO_BEACON_PINK_YELLOW, WO_BEACON_PINK_BLUE, WO_BEACON_BLUE_PINK, WO_BEACON_YELLOW_BLUE, WO_BEACON_BLUE_YELLOW };
  float sumOfWeights = 0;
  
  for(auto& p : particles()) {
	p.w = 1;
	for(int i = 0; i < 6; ++i){
	  if(checkBeaconVisibility(beacons,i,p)){
        
   	  	float meanDistance = sqrt((beacons[i].x - p.x) * (beacons[i].x - p.x) + (beacons[i].y - p.y) * (beacons[i].y - p.y));
		    float measurementDistance;
		  /*  if (i==0){
			    auto &object = cache_.world_object->objects_[WO_BEACON_YELLOW_PINK];
          measurementDistance = object.visionDistance;
		    }
		    else if (i==1){
			    auto &object = cache_.world_object->objects_[WO_BEACON_PINK_YELLOW];
          measurementDistance = object.visionDistance;
		    }
		    else if (i==2){
			    auto &object = cache_.world_object->objects_[WO_BEACON_PINK_BLUE];
           measurementDistance = object.visionDistance;
		    }
		    else if (i==3){
			    auto &object = cache_.world_object->objects_[WO_BEACON_BLUE_PINK];
           measurementDistance = object.visionDistance;
		    }
		    else if (i==4){
			    auto &object = cache_.world_object->objects_[WO_BEACON_YELLOW_BLUE];
           measurementDistance = object.visionDistance;
		    }
		    else if (i==5){
			    auto &object = cache_.world_object->objects_[WO_BEACON_BLUE_YELLOW];
           measurementDistance = object.visionDistance;
		    } *//*
        auto &object = cache_.world_object->objects_[beaconColors[i]];
        measurementDistance = object.visionDistance; 
        cout << i << " sensor "<<measurementDistance<<"dist "<< meanDistance<<endl;
      	p.w *= gaussianProbability(measurementDistance, meanDistance, 10000);
      
	    }
	 }
   cout << "prob "<< p.w <<endl;  
 }

 // for(auto& p : particles()) {
  //  p.w /= sumOfWeights;
 // }
  return sumOfWeights / numParticles;
}
		   */

void ParticleFilter::resampleByImportance() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::vector<float> particleWeights(numParticles);

  // Retrieve Weights
  for(int index = 0; index < numParticles; index++) {
    particleWeights[index] = particles()[index].w;
  }
  
  // Create sampled particles vector
  std::vector<Particle> resampledParticles(numParticles);  
  std::piecewise_constant_distribution<> distribution(particleIndices.begin(), particleIndices.end(), particleWeights.begin());
  for(int index = 0; index < (numParticles-noiseParticles); index++) {
    Particle particle = particles()[distribution(gen)];
    resampledParticles[index].x = particle.x;
    resampledParticles[index].y = particle.y;
    resampledParticles[index].t = particle.t;
    resampledParticles[index].w = 600;
  }

  // Copy sampled particles back
  for(int index = 0; index < (numParticles-noiseParticles); index++) {
    particles()[index] = resampledParticles[index];
  }
  
  for(int index = (numParticles-noiseParticles); index < numParticles; index++) {
    Particle particle;    
    particle.x = rand_.sampleN(0, 750);
    particle.y = rand_.sampleN(0, 350);
    particle.t = rand_.sampleN(0, M_PI / 2);
    particle.w = 600;
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
 // const auto& disp = cache_.odometry->displacement;
 // log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);

  propagateToNext();
  createParticleWeights();
  resampleByImportance(); 

  // Generate random particles for demonstration
 /* particles().resize(100);
  auto frame = cache_.frame_info->frame_id;
  for(auto& p : particles()) {
    p.x = rand_.sampleN(frame * 5, 250);
    p.y = rand_.sampleN(0, 250);
    p.t = rand_.sampleN(0, M_PI / 4);
    p.w = rand_.sampleU();
  }*/
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
    for(const auto& p : particles()) {
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    
    if(particles().size() > 0)
      mean_ /= particles().size();
    cout<<mean_.rotation<<endl;
    dirty_ = false;
  }
  return mean_;
}

