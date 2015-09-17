import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 5.0:
      memory.speech.say("ready to play")
      self.finish()

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 2.5:
        self.finish()

  class Walk(Node):    
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class SidewaysWalk(Node):
    def run(self):
      commands.setWalkVelocity(0,0.5,0)

  class TurningWalk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0.175)

  class InplaceTurningWalk(Node):
    def __init__(self):
      self.distanceMap = {core.WO_BEACON_YELLOW_BLUE:0, core.WO_BEACON_BLUE_YELLOW:0, core.WO_BEACON_BLUE_PINK:0, core.WO_BEACON_PINK_BLUE:0, core.WO_BEACON_YELLOW_PINK:0, core.WO_BEACON_PINK_YELLOW:0}
      self.toPan = 0
      super(self.__class__, self).__init__()

    def updateDistanceMap(self,key):
      beacon = memory.world_objects.getObjPtr(key)      
      if beacon:
        self.distanceMap[key] = max(self.distanceMap[key], beacon.visionDistance)

    def run(self):
    #  memory.speech.say("round and round")
      commands.setWalkVelocity(0,0,0.25)
      self.updateDistanceMap(core.WO_BEACON_YELLOW_BLUE)
      self.updateDistanceMap(core.WO_BEACON_BLUE_YELLOW)
      self.updateDistanceMap(core.WO_BEACON_YELLOW_PINK)
      self.updateDistanceMap(core.WO_BEACON_PINK_YELLOW)
      self.updateDistanceMap(core.WO_BEACON_YELLOW_BLUE)
      self.updateDistanceMap(core.WO_BEACON_BLUE_YELLOW)
      if self.getTime() > 20.0:
        print self.distanceMap
        memory.speech.say("done attempting to detect beacons")
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    stand = self.Stand()
    walk = self.Walk()
    sit = pose.Sit()
    off = self.Off()
    self.trans(self.Stand(), C, self.InplaceTurningWalk(), C, sit, C, off)

