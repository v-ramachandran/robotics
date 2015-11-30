import memory, pose, commands, cfgstiff, geometry, mem_objects, core, math
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
      if self.getTime() > 2.0:
        self.finish()
  
 
  
  class Initialize(Node):
    def run(self):
      core.localizationC.reInit()
      self.finish()
  
	class Walking(Node):
		def run(self):
			commands.setWalkVelocity(0.5, 0, 0)
			self.finish()
 

  class HeadScan(Node):
    def __init__(self):
      self.toTurn = math.pi / 96
      self.toPan = 0
      self.threshold = 0
      self.thresholdUnits = 1
      super(self.__class__, self).__init__()

    def run(self):
      if (core.joint_values[core.HeadPan] >= ((math.pi / 3) - 0.05)):
        self.toTurn = (-1)*(math.pi / 96) 
      elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 3) + 0.05)):
        self.toTurn = (math.pi / 96)
      
#      if self.getTime() >= self.threshold:
#        self.threshold = self.thresholdUnits + self.threshold
      new_value = self.toPan + self.toTurn
      max_cap = max(new_value, (-1)*(math.pi/3))
      min_cap = min(max_cap, math.pi / 3)
      self.toPan = min_cap
      commands.setHeadPan(self.toPan, target_time = 0.75)

      if self.getTime() >= 15.0:
        self.toTurn = math.pi / 96
        self.toPan = 0
        self.threshold = 0
        self.thresholdUnits = 1
        mypos = mem_objects.world_objects[memory.robot_state.WO_SELF]
        currentpos = geometry.Point2D(mypos.loc.x, mypos.loc.y)
        center = geometry.Point2D(0,0)
        bearing = currentpos.getBearingTo(center, mypos.orientation)
        distance = currentpos.getDistanceTo(center)
        memory.speech.say("done scanning")
        # print "scan distance ",distance
        commands.setHeadPan(0, target_time = 0.5)
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
		self.trans(self.Stand(), C, self.HeadScan(), C, self.Walk(), C, off)
 #  self.trans(self.Stand(), C, self.Walk(), T(5.0), self.Stand(), C, sit, C, off)
 #   self.trans(stand, C, self.Initialize(), C, self.InplaceTurningWalk(), C, arrive_at_center, C, self.Stand(), C, off)
