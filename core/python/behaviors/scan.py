import memory, pose, commands, cfgstiff, core, mem_objects, math, cfgpose
from task import Task
from memory import *
from state_machine import *

class Playing(StateMachine):
  class Declare(Node):
    def run(self):
      memory.speech.say("Starting up Kicker")
      self.finish()

  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 2.0:
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  class Scan(Node):
    def __init__(self):
      self.toTurn = math.pi / 96
      self.toPan = 0
      super(self.__class__, self).__init__()

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if self.getTime() > 15.0:
        commands.setHeadPan(0, target_time = 0.5)
        self.finish()
      else:
        if (core.joint_values[core.HeadPan] >= ((math.pi / 3) - 0.05)):
          self.toTurn = (-1)*(math.pi / 96) 
        elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 3) + 0.05)):
          self.toTurn = (math.pi / 96)
        new_value = self.toPan + self.toTurn
        max_cap = max(new_value, (-1)*(math.pi/3))
        min_cap = min(max_cap, math.pi / 3)
        self.toPan = min_cap
        commands.setHeadPan(self.toPan, target_time = 0.75)

  def setup(self):
    stand = self.Stand()
    scan = self.Scan()
    off = self.Off()
    self.trans(stand, C, scan)
    self.trans(scan, C, off)

