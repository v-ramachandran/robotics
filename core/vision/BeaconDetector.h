#pragma once

#include <vision/ObjectDetector.h>
#include <vision/ImageProcessor.h>
#include <vision/structures/TreeNode.h>
#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <algorithm>
#include <iostream>

class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  vector<int> findColoredBeacon(Color color1, Color color2, std::map<Color, struct DisjointSet> colorDisjointSets, ImageProcessor * processor);
  void findBeacons(std::map<Color, struct DisjointSet> colorDisjointSets, ImageProcessor * processor);
  bool checkColorUpperPixels(ImageProcessor * processor, int leftX, int rightX, int height);
 private:
  TextLogger* textlogger;
};
