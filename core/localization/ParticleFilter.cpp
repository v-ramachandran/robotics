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

const float ParticleFilter::distance(Particle X, Particle Y) const {
	return sqrt((X.x - Y.x)*(X.x - Y.x) + (X.y - Y.y)*(X.y - Y.y));
}

const Particle ParticleFilter::kMeans() const {
  
  Particle clusterCenters[4];
  for(int i=0; i<4; ++i){
    srand (time(NULL));
    int index = rand() % particles().size();
    clusterCenters[i].x = particles()[i].x;
    clusterCenters[i].y = particles()[i].y;
		clusterCenters[i].t = particles()[i].t;
  }
  map<int, vector<int>> clusters;
	int count = 20;
	int threshold = 0.5;
  while(count){
		for(int i=0; i< particles().size(); ++i){
			int center = 0;
			float centerDistance = distance(particles()[i], clusterCenters[0]);
			for(int j=1; j<4; ++j){
				float dist = distance(particles()[i], clusterCenters[j]);
				if(dist < centerDistance){
					center = j;
					centerDistance = dist;
				}
			}
			clusters[center].push_back(i);
		}
		count --;
		bool convergence = true;
		for(int i=0; i<4; ++i){
			float sumX = 0;
			float sumY = 0;
			float sumT = 0;
			int count = 0;
			for(int j=0; j<clusters[i].size(); ++j){
				sumX += particles()[clusters[i][j]].x;
				sumY += particles()[clusters[i][j]].y;
				sumT += particles()[clusters[i][j]].t;
			}
			if(abs(clusterCenters[i].x - sumX / clusters[i].size()) > threshold || abs(clusterCenters[i].y - sumY / clusters[i].size()) > threshold)
				convergence= false;
			clusterCenters[i].x = sumX / clusters[i].size();
			clusterCenters[i].y = sumY / clusters[i].size();
			clusterCenters[i].t = sumT / clusters[i].size();
			
		}
		if (convergence)
			break;
  }

	int maxClusterSize = 0;
	int maxClusterIndex = -1;
	for(int i = 0; i< 4; ++i){
		if(clusters[i].size() > maxClusterSize){
			maxClusterSize = clusters[i].size();
			maxClusterIndex = i;
		}
	}
	return clusterCenters[maxClusterIndex];
	
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
       p.t = p.t + rand_.sampleN(0, 0.05);
//    }
		float meanX = disp.translation.x + p.x;
		p.x = disp.translation.x * cos(p.t) + disp.translation.y * sin(p.t) + p.x;
		float meanY = disp.translation.y  + p.y;
		p.y = disp.translation.x * sin(p.t)+ disp.translation.y * cos(p.t) + p.y;

//    if(!isEqual(disp.translation.x,0) || !isEqual(disp.translation.y,0)){
      p.x += rand_.sampleN(0, 5);
      p.y += rand_.sampleN(0, 5);
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
  int orientationPenaltyScale = 100;
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
        // cout << "mean measurement distance " << meanDistance << " " << measurementDistance << endl;
        float probability = gaussianProbability(measurementDistance, meanDistance, 20000);
        float penalty = min((maxPenalty - maxPenaltyPadding),(1-probability)*100); // p=0 -> 50, p=1 -> 0
        p.w = p.w - penalty;
        //cout << i << "|" << p.x << "|" << object.loc.x << "|" << p.y << "|" << object.loc.y << "|" << object.loc.x << "|" << object.loc.y << "|" << probability << "|" << penalty << "|" << p.w << endl;
        float rotationProbability = gaussianProbability(object.visionBearing, Point2D(p.x,p.y).getBearingTo(object.loc, p.t), 0.01);
        p.w = p.w - orientationPenaltyScale*(1-rotationProbability);
        cout << i << "|" << object.visionBearing << "|" << Point2D(p.x,p.y).getBearingTo(object.loc, p.t) << "|" << rotationProbability << "|" << orientationPenaltyScale*(1-rotationProbability) << "|" << p.w << endl;
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
  
  Pose2D mean = mean_;
  float y = mean.y;
  float x = mean.x;
  for(int index = (numParticles-noiseParticles); index < numParticles; index++) {
    Particle particle;    
    particle.x = rand_.sampleN(x, 25);
    particle.y = rand_.sampleN(y, 25);
    particle.t = rand_.sampleN(0, M_PI / 2);
    particle.w = 600;
  }
}


void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  propagateToNext();
  createParticleWeights();
  resampleByImportance(); 
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
 /*   for(const auto& p : particles()) {
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    
    if(particles().size() > 0)
      mean_ /= particles().size();*/
		Particle clusterCentroid = kMeans();
		mean_.translation = T(clusterCentroid.x, clusterCentroid.y);
		mean_.rotation = clusterCentroid.t;		
    cout<<mean_.rotation<<endl;
    dirty_ = false;
  }
  return mean_;
}

