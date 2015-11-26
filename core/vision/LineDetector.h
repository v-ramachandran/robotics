#pragma once

#include <vision/ObjectDetector.h>
#include <vision/ImageProcessor.h>
#include <vision/structures/TreeNode.h>
#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <vision/structures/LinePoint.h>
#include <algorithm>
#include <iostream>

class TextLogger;

/// @ingroup vision
class LineDetector : public ObjectDetector {
 public:
	vector<LinePoint> linePoints; 	

  LineDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findLine(ImageProcessor * processor);
  void findLinePointCandidates(ImageProcessor * processor);
 private:
  TextLogger* textlogger;
};
