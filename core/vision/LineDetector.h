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
class LineDetector : public ObjectDetector {
 public:
  LineDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findLine();
  void findLinePointCandidates();
 private:
  TextLogger* textlogger;
};
