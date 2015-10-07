import core, memory
import pose, commands, cfgstiff, cfgpose
import mem_objects
from task import Task
from state_machine import *
import random
import math

class BlockLeft(Node):
  def run(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    print "BLOCKING LEFT", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
    UTdebug.log(15, "Blocking left")

class BlockRight(Node):
  def run(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    print "BLOCKING RIGHT", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
    UTdebug.log(15, "Blocking right")

class BlockCenter(Node):
  def run(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    print "BLOCKING CENTER", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
    UTdebug.log(15, "Blocking center")

class Blocker(Node):
  def __init__(self):
    self.timesUnseen = 0
    super(self.__class__, self).__init__()  
  
  def compute_y_at_origin(self, point1, point2):
    return 0

  def run(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    #print ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
    #futureX = ball.distance * math.cos(ball.bearing) (1 + 3*ball.absVel.x)
    #print futureX
    if ball.seen:
      self.timesUnseen = 0
      commands.setHeadPan(ball.visionBearing, 0.25)
      if (ball.distance < 500) and (ball.absVel.x < -25):
        UTdebug.log(15, "Ball is close, blocking!")
        if (ball.bearing > 30 * core.DEG_T_RAD):
          if(ball.loc.x >= 0):
            print "---------left", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y, ball.endLoc.x, ball.endLoc.y
            choice = "left"
          else:
            print "---------right", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y, ball.endLoc.x, ball.endLoc.y
            choice = "right"
        elif (ball.bearing < -30 * core.DEG_T_RAD):
          if ball.loc.x >= 0:
            print "-----------right", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y, ball.endLoc.x, ball.endLoc.y
            choice = "right"
          else:
            print "-----------left", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y, ball.endLoc.x, ball.endLoc.y
            choice = "left"
        else:
          print "---------center", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y, ball.endLoc.x, ball.endLoc.y
          choice = "center"
        self.postSignal(choice)
    else:
      self.timesUnseen = self.timesUnseen + 1
    if self.timesUnseen > 10:
      commands.setHeadPan(0,0.1)

class BlockingLeft(StateMachine):
  class PerformLogistics(Node):
    def run(self):
      ball = mem_objects.world_objects[core.WO_BALL]
      print "BLOCKING LEFT", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
      UTdebug.log(15, "Blocking left")
      self.finish()

  def setup(self):
    self.trans(pose.PoseSequence(cfgpose.goalieSimBlockLeft, 0.3), T(3.0), self.PerformLogistics(), C, pose.PoseSequence(cfgpose.sittingPoseV3, 1.0))

class BlockingRight(StateMachine):
  class PerformLogistics(Node):
    def run(self):
      ball = mem_objects.world_objects[core.WO_BALL]
      print "BLOCKING RIGHT", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
      UTdebug.log(15, "Blocking right")
      self.finish()

  def setup(self):
    self.trans(pose.PoseSequence(cfgpose.goalieSimBlockRight, 0.3), T(3.0), self.PerformLogistics(), C, pose.PoseSequence(cfgpose.sittingPoseV3, 1.0))

class BlockingCenter(StateMachine):
  
  class PerformLogistics(Node):
    def run(self):
      ball = mem_objects.world_objects[core.WO_BALL]
      print "BLOCKING CENTER", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
      UTdebug.log(15, "Blocking center")
      self.finish()
    
  def setup(self):
    self.trans(pose.PoseSequence(cfgpose.goalieSimBlockCenter, 0.3), T(3.0), self.PerformLogistics(), C, pose.PoseSequence(cfgpose.sittingPoseV3, 1.0))

class Playing(LoopingStateMachine):
  def setup(self):
    blocker = Blocker()
    self.trans(blocker, S("left"), BlockingLeft(), T(5), blocker)
    self.trans(blocker, S("right"), BlockingRight(), T(5), blocker)
    self.trans(blocker, S("center"), BlockingCenter(), T(5), blocker)
    #self.trans(pose.Sit(), C, BlockingLeft(), T(5.0), BlockingRight(), T(5.0), BlockingCenter(), T(5.0), pose.Sit())
