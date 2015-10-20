#pragma once

#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>
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
    void propagateToNext();
    bool isEqual(float x , float y);
    float createParticleWeights();
    float gaussianProbability(float x, float mean, float var);
    bool checkBeaconVisibility(Point2D beaconLoc, Particle p );
    void resampleByImportance();

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    Random rand_;
    int xMin = -250;
    int xMax = 250;
    int yMin = -125;
    int yMax = 125;
    int numParticles= 1000;
    int noiseParticles = 1;
    std::vector<int> particleIndices;
    mutable Pose2D mean_;
    mutable bool dirty_;
};
