#pragma once

#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>
#include <map>
#include <math/Pose2D.h>
#include <common/Random.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>

class ParticleFilter {
  public:
    ParticleFilter(MemoryCache& cache, TextLogger*& tlogger);
    void init(Point2D loc, float orientation);
    void processFrame();
    const Pose2D& pose() const;
    inline const std::vector<Particle>& particles() const {
      return cache_.localization_mem->particles;
    }

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }
    void filter();
    void propagateToNext();
    bool isEqual(float x , float y);
    float createParticleWeights();
    float gaussianProbability(float x, float mean, float var);
    bool checkBeaconVisibility(Point2D beaconLoc, Particle p );
    void resampleByImportance(float wSlow, float wFast);
    void resampleByImportance();
		const float distance(Particle X, Particle Y) const;
		const Particle kMeans() const;

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    Random rand_;
    int xMin = -250;
    int xMax = 250;
    int yMin = -125;
    int yMax = 125;
    int numParticles= 1000;
    int noiseParticles = 0;
    float alphaFast = 0.9;
    float alphaSlow = 0.003;
    float wSlow = 0.0;
    float wFast = 0.0;
    std::vector<int> particleIndices;
    mutable Pose2D mean_;
    mutable bool dirty_;
};
