import memory, pose, commands, cfgstiff, geometry, mem_objects, core, math
from task import Task
from memory import *
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
      if self.getTime() > 2.0:
        memory.speech.say("I'm localizing")
        self.postSignal("initialize")
  
  class Initialize(Node):
    def run(self):
      core.localizationC.reInit()
      memory.speech.say("I'm localizing")
      self.postSignal("localize")

  class WalkWithHeadScan(Node):
    def __init__(self):
      self.toTurn = math.pi / 96
      self.toPan = 0
      super(self.__class__, self).__init__()

    def run(self):
      if (core.joint_values[core.HeadPan] >= ((math.pi / 6) - 0.05)):
        self.toTurn = (-1)*(math.pi / 96) 
      elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 6) + 0.05)):
        self.toTurn = (math.pi / 96)
      
      new_value = self.toPan + self.toTurn
      max_cap = max(new_value, (-1)*(math.pi/6))
      min_cap = min(max_cap, math.pi / 6)
      self.toPan = min_cap
      commands.setHeadPan(self.toPan, target_time = 0.75)

      boundary = world_objects.getObjPtr(core.WO_BOTTOM_BOUNDARY_SEGMENT)
      beacon = world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK)
      topboundary = world_objects.getObjPtr(core.WO_TOP_BOUNDARY_SEGMENT)
      if(topboundary.seen):
        print "TOP ", topboundary.boundarySegment.totalPoints, topboundary.boundarySegment.minDistance, topboundary.boundarySegment.maxDistance, topboundary.boundarySegment.averageDistance
      print "BOTTOM ",boundary.boundarySegment.totalPoints, boundary.boundarySegment.minDistance, boundary.boundarySegment.maxDistance, boundary.boundarySegment.averageDistance 
      condition_1 = boundary.seen and boundary.boundarySegment.maxDistance <= 250 and boundary.boundarySegment.totalPoints >= 10 and boundary.boundarySegment.averageDistance <= 250
      condition_2 = boundary.seen and boundary.boundarySegment.totalPoints >=100 and boundary.boundarySegment.averageDistance <= 300
      if condition_2 or condition_1: #and math.fabs(boundary.boundarySegment.averageDistance - ((boundary.boundarySegment.minDistance + boundary.boundarySegment.maxDistance)/2)) <= 50:
        memory.speech.say("I am at a boundary")
        commands.setWalkVelocity(0,0,0)
        commands.setHeadPan(0, target_time = 0.5)
        self.postSignal("standby")
      else:
        commands.setWalkVelocity(0.45, 0, 0)
        
  class HeadScan(Node):
    def __init__(self):
      self.toTurn = math.pi / 96
      self.toPan = 0
      super(self.__class__, self).__init__()

    def run(self):
      if (core.joint_values[core.HeadPan] >= ((math.pi / 4) - 0.05)):
        self.toTurn = (-1)*(math.pi / 96) 
      elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 4) + 0.05)):
        self.toTurn = (math.pi / 96)
      
      new_value = self.toPan + self.toTurn
      max_cap = max(new_value, (-1)*(math.pi/4))
      min_cap = min(max_cap, math.pi / 4)
      self.toPan = min_cap
      commands.setHeadPan(self.toPan, target_time = 1)

      if self.getTime() >= 15.0:
        self.toTurn = math.pi / 96
        self.toPan = 0
        memory.speech.say("Walking to boundary")
        commands.setHeadPan(0, target_time = 0.5)
        self.postSignal("walk")

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    stand = self.Stand()
    initialize = self.Initialize()
    headscan = self.HeadScan()
    walkwithheadscan = self.WalkWithHeadScan()
    off = self.Off()

    self.trans(stand, S("initialize"), initialize)
    self.trans(initialize, S("localize"), headscan)
    self.trans(headscan, S("walk"), walkwithheadscan)
    self.trans(walkwithheadscan, S("standby"), self.Stand())
