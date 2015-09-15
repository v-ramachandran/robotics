#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <vision/Classifier.h>
#include <common/RobotCalibration.h>
#include <vision/structures/BallCandidate.h>
#include <vision/structures/TreeNode.h>
#include <math/Pose3D.h>
#include <cmath>

class BeaconDetector;

/// @ingroup vision
class ImageProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    void processFrame();
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    Classifier* classifier_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(RobotCalibration);
    void enableCalibration(bool value);
    void updateTransform();
    bool tiltAngleTest(struct TreeNode * treenode, float threshold);
    bool isSquare(struct TreeNode * treeNode);
    bool isAtleastMinimumSize(struct TreeNode * treeNode);
    bool isCircularArea(struct TreeNode * treeNode);
    bool hasMinimumArea(struct TreeNode * treeNode);
    bool goalAspectRatioTest(struct TreeNode * node);
    std::vector<BallCandidate*> getBallCandidates();
    std::vector<TreeNode*> getGoalCandidates();
    BallCandidate* getBestBallCandidate();
    struct TreeNode* getBestGoalCandidate();
    void getBlobNodes();
    bool isImageLoaded();
    void detectBall();
    void findBall(int& imageX, int& imageY);
  private:
    int getTeamColor();
    double getCurrentTime();

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;
    
    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;
    std::map<Color, struct DisjointSet> colorDisjointSets;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;

    RobotCalibration* calibration_;
    bool enableCalibration_;
    BeaconDetector* beacon_detector_;
};

#endif
