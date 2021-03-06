#include "InverseKinematics.h"
#include <common/InterfaceInfo.h>

bool InverseKinematics::calcLegJoints(const Pose3D& positionLeft, const Pose3D& positionRight, Joints jointAngles, const RobotDimensions& robotDimensions, float ratio) {
  bool reachable = true;
  if(!calcLegJoints(positionLeft, jointAngles, true, robotDimensions))
    reachable = false;
  if(!calcLegJoints(positionRight, jointAngles, false, robotDimensions))
    reachable = false;
  Range<float> clipping(0.0f, 1.0f);
  ratio = clipping.limit(ratio);
  // the hip joints of both legs must be equal, so it is computed as weighted mean and the foot positions are
  // recomputed with fixed joint0 and left open foot rotation (as possible failure)
  float joint0 = jointAngles[LHipYawPitch] * ratio + jointAngles[RHipYawPitch] * (1 - ratio);
  if(!calcLegJoints(positionLeft, jointAngles, joint0, true, robotDimensions))
    reachable = false;
  if(!calcLegJoints(positionRight, jointAngles, joint0, false, robotDimensions))
    reachable = false;
  return reachable;
}
  
bool InverseKinematics::calcLegJoints(const Pose3D& position, Joints jointAngles, bool left, const RobotDimensions& robotDimensions) {
  Pose3D target(position);
  Joint firstJoint(left ? LHipYawPitch : RHipYawPitch);
  int sign(left ? -1 : 1);
  target.translation.y += (float) robotDimensions.values_[RobotDimensions::lengthBetweenLegs] / 2.f * sign; // translate to origin of leg
  // rotate by 45° around origin for Nao
  // calculating sqrtf(2) is faster than calculating the resp. rotation matrix with getRotationX()
  static const float sqrt2_2 = sqrtf(2.0f) * 0.5f;
  RotationMatrix rotationX_pi_4 = RotationMatrix(Vector3<float>(1, 0, 0), Vector3<float>(0, sqrt2_2, sqrt2_2 * sign), Vector3<float>(0, sqrt2_2 * -sign, sqrt2_2));
  target.translation = rotationX_pi_4 * target.translation;
  target.rotation = rotationX_pi_4 * target.rotation;

  target = target.invert(); // invert pose to get position of body relative to foot

  // use geometrical solution to compute last three joints
  float length = target.translation.abs();
  float sqrLength = length * length;
  float upperLeg = robotDimensions.values_[RobotDimensions::upperLegLength];
  float sqrUpperLeg = upperLeg * upperLeg;
  float lowerLeg = robotDimensions.values_[RobotDimensions::lowerLegLength];
  float sqrLowerLeg = lowerLeg * lowerLeg;
  float cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2 * lowerLeg * length);
  float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * upperLeg * lowerLeg);

  // clip for the case of unreachable position
  const Range<float> clipping(-1.0f, 1.0f);
  bool reachable = true;
  if(!clipping.isInside(cosKnee) || clipping.isInside(cosLowerLeg))
  {
    cosKnee = clipping.limit(cosKnee);
    cosLowerLeg = clipping.limit(cosLowerLeg);
    reachable = false;
  }
  float joint3 = M_PI - acosf(cosKnee); // implicitly solve discrete ambiguousness (knee always moves forward)
  float joint4 = -acosf(cosLowerLeg);
  joint4 -= atan2f(target.translation.x, Vector2<float>(target.translation.y, target.translation.z).abs());
  float joint5 = atan2f(target.translation.y, target.translation.z) * sign;

  // calulate rotation matrix before hip joints
  RotationMatrix hipFromFoot;
  hipFromFoot.rotateX(joint5 * -sign);
  hipFromFoot.rotateY(-joint4 - joint3);

  // compute rotation matrix for hip from rotation before hip and desired rotation
  RotationMatrix hip = hipFromFoot.invert() * target.rotation;

  // compute joints from rotation matrix using theorem of euler angles
  // see http://www.geometrictools.com/Documentation/EulerAngles.pdf
  // this is possible because of the known order of joints (z, x, y seen from body resp. y, x, z seen from foot)
  float joint1 = asinf(-hip[2].y) * -sign;
  joint1 -= M_PI_4; // because of the 45°-rotational construction of the Nao legs
  float joint2 = -atan2f(hip[2].x, hip[2].z);
  float joint0 = atan2f(hip[0].y, hip[1].y) * -sign;

  // set computed joints in jointAngles
  jointAngles[firstJoint + 0] = joint0;
  jointAngles[firstJoint + 1] = joint1;
  jointAngles[firstJoint + 2] = joint2;
  jointAngles[firstJoint + 3] = joint3;
  jointAngles[firstJoint + 4] = joint4;
  jointAngles[firstJoint + 5] = joint5;

  return reachable;
}
  
bool InverseKinematics::calcLegJoints(const Pose3D& position, Joints jointAngles, float joint0, bool left, const RobotDimensions& robotDimensions) {
  Pose3D target(position);
  Joint firstJoint(left ? LHipYawPitch : RHipYawPitch);
  const int sign(left ? -1 : 1);
  target.translation.y += robotDimensions.values_[RobotDimensions::lengthBetweenLegs] / 2 * sign; // translate to origin of leg
  target = Pose3D().rotateZ(joint0 * -sign).rotateX(M_PI_4 * sign).conc(target); // compute residual transformation with fixed joint0

  // use cosine theorem and arctan to compute first three joints
  float length = target.translation.abs();
  float sqrLength = length * length;
  float upperLeg = robotDimensions.values_[RobotDimensions::upperLegLength];
  float sqrUpperLeg = upperLeg * upperLeg;
  float lowerLeg = robotDimensions.values_[RobotDimensions::lowerLegLength];
  float sqrLowerLeg = lowerLeg * lowerLeg;
  float cosUpperLeg = (sqrUpperLeg + sqrLength - sqrLowerLeg) / (2 * upperLeg * length);
  float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * upperLeg * lowerLeg);
  // clip for the case that target position is not reachable
  const Range<float> clipping(-1.0f, 1.0f);
  bool reachable = true;
  if(!clipping.isInside(cosKnee) || clipping.isInside(upperLeg))
  {
    cosKnee = clipping.limit(cosKnee);
    cosUpperLeg = clipping.limit(cosUpperLeg);
    reachable = false;
  }
  float joint1 = target.translation.z == 0.0f ? 0.0f : atanf(target.translation.y / -target.translation.z) * sign;
  float joint2 = -acos(cosUpperLeg);
  joint2 -= atan2(target.translation.x, Vector2<float>(target.translation.y, target.translation.z).abs() * -sgn(target.translation.z));
  float joint3 = M_PI - acos(cosKnee);
  RotationMatrix beforeFoot = RotationMatrix().rotateX(joint1 * sign).rotateY(joint2 + joint3);
  joint1 -= M_PI_4; // because of the strange hip of Nao

  // compute joints from rotation matrix using theorem of euler angles
  // see http://www.geometrictools.com/Documentation/EulerAngles.pdf
  // this is possible because of the known order of joints (y, x, z) where z is left open and is seen as failure
  RotationMatrix foot = beforeFoot.invert() * target.rotation;
  float joint5 = asin(-foot[2].y) * -sign * -1;
  float joint4 = -atan2(foot[2].x, foot[2].z) * -1;
  //float failure = atan2(foot[0].y, foot[1].y) * sign;

  // set computed joints in jointAngles
  jointAngles[firstJoint + 0] = joint0;
  jointAngles[firstJoint + 1] = joint1;
  jointAngles[firstJoint + 2] = joint2;
  jointAngles[firstJoint + 3] = joint3;
  jointAngles[firstJoint + 4] = joint4;
  jointAngles[firstJoint + 5] = joint5;

  return reachable;
}

bool InverseKinematics::calcArmJoints(const Pose3D& left, const Pose3D& right, Joints jointAngles, const RobotDimensions& theRobotDimensions) {
  const Vector3<float> leftDir = left.rotation * Vector3<float>(0, -1, 0),
                  rightDir = right.rotation * Vector3<float>(0, 1, 0);

  //transform to "shoulder"-coordinate-system
  Vector3<float> leftTarget = left.translation - Vector3<float>(theRobotDimensions.values_[RobotDimensions::armOffset1],
                         theRobotDimensions.values_[RobotDimensions::armOffset2],
                         theRobotDimensions.values_[RobotDimensions::armOffset3]),
                         rightTarget = right.translation - Vector3<float>(theRobotDimensions.values_[RobotDimensions::armOffset1],
                                       -theRobotDimensions.values_[RobotDimensions::armOffset2],
                                       theRobotDimensions.values_[RobotDimensions::armOffset3]);

  //avoid straigt arm
  static const float maxLength = (theRobotDimensions.values_[RobotDimensions::upperArmLength] + theRobotDimensions.values_[RobotDimensions::lowerArmLength]) * 0.9999f;
  if(leftTarget.squareAbs() >= sqr(maxLength))
    leftTarget.normalize(maxLength);

  if(rightTarget.squareAbs() >= sqr(maxLength))
    rightTarget.normalize(maxLength);

  bool res1, res2;
  res1 = calcArmJoints(leftTarget, leftDir, 1, jointAngles, theRobotDimensions);
  res2 = calcArmJoints(rightTarget, rightDir, -1, jointAngles, theRobotDimensions);

  return res1 && res2;
}

bool InverseKinematics::calcArmJoints(Vector3<float> target, Vector3<float> targetDir, int side, Joints jointAngles, const RobotDimensions& theRobotDimensions) {
  //hacked mirror
  target.y *= (float)side;
  targetDir.y *= (float)side;

  const int offset = side == -1 ? RShoulderPitch : LShoulderPitch;

  Vector3<float> elbow;
  if(!calcElbowPosition(target, targetDir, side, elbow, theRobotDimensions))
    return false;

  calcJointsForElbowPos(elbow, target, jointAngles, offset, theRobotDimensions);

  return true;
}

bool InverseKinematics::calcElbowPosition(Vector3<float> &target, const Vector3<float> &targetDir, int side, Vector3<float> &elbow, const RobotDimensions& theRobotDimensions) {
  const Vector3<float> M1(0, 0, 0); //shoulder
  const Vector3<float> M2(target); //hand
  const float r1 = theRobotDimensions.values_[RobotDimensions::upperArmLength];
  const float r2 = theRobotDimensions.values_[RobotDimensions::lowerArmLength];
  const Vector3<float> M12 = M2 - M1;

  Vector3<float> n = target;
  n.normalize();

  //center of intersection circle of spheres around shoulder and hand
  const Vector3<float> M3 = M1 + M12 * ((sqr(r1) - sqr(r2)) / (2 * M12.squareAbs()) + 0.5f);

  //calculate radius of intersection circle
  const Vector3<float> M23 = M3 - M2;
  float diff = sqr(r2) - M23.squareAbs();
  const float radius = sqrt(diff);

  //determine a point on the circle
  const bool specialCase = n.x == 1 && n.y == 0 && n.z == 0 ? true : false;
  const Vector3<float> bla(specialCase ? 0.0f : 1.0f, specialCase ? 1.0f : 0.0f, 0.0f);
  const Vector3<float> pointOnCircle = M3 + (n ^ bla).normalize(radius);

  //find best point on circle
  float angleDiff = M_PI * 2.0f / 3.0f;
  Vector3<float> bestMatch = pointOnCircle;
  float bestAngle = 0.0f;
  float newBestAngle = bestAngle;
  float bestQuality = -2.0f;

  Vector3<float> tDir = targetDir;
  tDir.normalize();

  const int offset = side == 1 ? 0 : 4;
  int iterationCounter = 0;
  const float maxAngleEpsilon = 1.0f * M_PI / 180.0f;
  while(2.0f * angleDiff > maxAngleEpsilon)
  {
    for(int i = -1; i <= 1; i++)
    {
      if(i == 0 && iterationCounter != 1)
        continue;

      iterationCounter++;

      const Pose3D elbowRotation(RotationMatrix(n, bestAngle + angleDiff * i));
      const Vector3<float> possibleElbow = elbowRotation * pointOnCircle;
      const Vector3<float> elbowDir = (M3 - possibleElbow).normalize();
      float quality = elbowDir * tDir;
      if(quality > bestQuality)
      {
        bestQuality = quality;
        bestMatch = possibleElbow;
        newBestAngle = bestAngle + angleDiff * i;
      }
    }
    angleDiff /= 2.0f;
    bestAngle = newBestAngle;
  }
  //printf("iterations %d\n", iterationCounter);
  if(bestQuality == -2.0f)
    return false;

  //special case of target-out-of-joints-limit problem
  float tAJR[NUM_JOINTS];
  calcJointsForElbowPos(bestMatch, target, tAJR, offset, theRobotDimensions);

  int jointInd = LShoulderPitch + offset + 1;
  float minVal = robot_joint_signs[jointInd] * minJointLimits[jointInd];
  if(tAJR[offset + 1] < minVal)
  {
    tAJR[offset + 1] = minVal;
    Pose3D shoulder2Elbow;
    shoulder2Elbow.translate(0, -theRobotDimensions.values_[RobotDimensions::upperArmLength], 0);
    shoulder2Elbow.rotateX(-(tAJR[offset + 1] - M_PI_2));
    shoulder2Elbow.rotateY(tAJR[offset + 0] + M_PI_2);
    Vector3<float> handInEllbow = shoulder2Elbow * target;

    handInEllbow.normalize(theRobotDimensions.values_[RobotDimensions::lowerArmLength]);
    target = shoulder2Elbow.invert() * handInEllbow;
    bestMatch = shoulder2Elbow.invert() * Vector3<float>(0, 0, 0);
  }

  elbow = bestMatch;
  return true;
}

void InverseKinematics::calcJointsForElbowPos(const Vector3<float> &elbow, const Vector3<float> &target, Joints jointAngles, int offset, const RobotDimensions& theRobotDimensions) {
  //set elbow position with the pitch/yaw unit in the shoulder
  jointAngles[offset + 0] = atan2f(elbow.z, elbow.x);
  jointAngles[offset + 1] = atan2f(elbow.y, sqrtf(sqr(elbow.x) + sqr(elbow.z)));

  //calculate desired elbow "klapp"-angle
  const float c = target.abs(),
              a = theRobotDimensions.values_[RobotDimensions::upperArmLength],
              b = theRobotDimensions.values_[RobotDimensions::lowerArmLength];

  //cosine theorem
  float cosAngle = (-sqr(c) + sqr(b) + sqr(a)) / (2.0f * a * b);
  if(cosAngle < -1.0f)
  {
//      ASSERT(cosAngle > -1.1);
    cosAngle = -1.0f;
  }
  else if(cosAngle > 1.0f)
  {
    assert(cosAngle < 1.1);
    cosAngle = 1.0f;
  }
  jointAngles[offset + 3] = acosf(cosAngle) - M_PI;

  //calculate hand in elbow coordinate system and calculate last angle
  Pose3D shoulder2Elbow;
  shoulder2Elbow.translate(0, -theRobotDimensions.values_[RobotDimensions::upperArmLength], 0);
  shoulder2Elbow.rotateX(-(jointAngles[offset + 1] - M_PI_2));
  shoulder2Elbow.rotateY(jointAngles[offset + 0] + M_PI_2);
  const Vector3<float> handInEllbow = shoulder2Elbow * target;

  jointAngles[offset + 2] = -(atan2(handInEllbow.z, handInEllbow.x) + M_PI_2);
  while(jointAngles[offset + 2] > M_PI)
    jointAngles[offset + 2] -= M_2_PI;
  while(jointAngles[offset + 2] < -M_PI)
    jointAngles[offset + 2] += M_2_PI;
}

void InverseKinematics::calcArmJoints(const Vector3<float>& position, const float elbowYaw, Joints jointAngles, bool left, const RobotDimensions& robotDimensions) {
  Joint firstJoint(left ? LShoulderPitch : RShoulderPitch);
  const int sign(left ? -1 : 1);
  const Vector3<float> pos(position - Vector3<float>(robotDimensions.values_[RobotDimensions::armOffset1], robotDimensions.values_[RobotDimensions::armOffset2] * -sign, robotDimensions.values_[RobotDimensions::armOffset3]));
  float& joint0 = jointAngles[firstJoint + 0];
  float& joint1 = jointAngles[firstJoint + 1];
  const float& joint2 = jointAngles[firstJoint + 2] = elbowYaw;
  float& joint3 = jointAngles[firstJoint + 3];

  // distance of the end effector position to the origin
  const float positionAbs = pos.abs();

  // the upper and lower arm form a triangle with the air line to the end effector position being the third edge. Elbow angle can be computed using cosine theorem
  const float actualUpperArmLength = Vector2<float>(robotDimensions.values_[RobotDimensions::upperArmLength], robotDimensions.values_[RobotDimensions::elbowOffsetY]).abs();
  float cosElbow = (sqr(actualUpperArmLength) + sqr(robotDimensions.values_[RobotDimensions::lowerArmLength]) - sqr(positionAbs)) / (2.0f * robotDimensions.values_[RobotDimensions::upperArmLength] * robotDimensions.values_[RobotDimensions::lowerArmLength]);
  // clip for the case of unreachable position
  cosElbow = Range<float>(-1.0f, 1.0f).limit(cosElbow);
  // elbow is streched in zero-position, hence M_PI - innerAngle
  joint3 = -(M_PI - acos(cosElbow));
  // compute temporary end effector position from known third and fourth joint angle
  const Pose3D tempPose = Pose3D(robotDimensions.values_[RobotDimensions::upperArmLength], robotDimensions.values_[RobotDimensions::elbowOffsetY] * -sign, 0).rotateX(joint2 * -sign).rotateZ(joint3 * -sign).translate(robotDimensions.values_[RobotDimensions::lowerArmLength], 0, 0);

  /* offset caused by third and fourth joint angle */                /* angle needed to realise y-component of target position */
  joint1 = atan2(tempPose.translation.y * sign, tempPose.translation.x) + asin(pos.y / Vector2<float>(tempPose.translation.x, tempPose.translation.y).abs()) * -sign;
  // refresh temporary endeffector position with known joint1
  const Pose3D tempPose2 = Pose3D().rotateZ(joint1 * -sign).conc(tempPose);

  /* first compensate offset from temporary position */       /* angle from target x- and z-component of target position */
  joint0 = -atan2(tempPose2.translation.z, tempPose2.translation.x) + atan2(pos.z, pos.x);
}
