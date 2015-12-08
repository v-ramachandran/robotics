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
        memory.speech.say("I'm initializing")
        self.postSignal("initialize")
  
  class Initialize(Node):
    def run(self):
      core.localizationC.reInit()
      memory.speech.say("I'm localizing")
      self.postSignal("localize")
        
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

      if self.getTime() >= 20.0:
        self.toTurn = math.pi / 96
        self.toPan = 0
     #   memory.speech.say("Walking to boundary")
        commands.setHeadPan(0, target_time = 0.5)
        self.postSignal("standby")

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
    off = self.Off()

    self.trans(stand, S("initialize"), initialize)
    self.trans(initialize, S("localize"), headscan)
    self.trans(headscan, S("standby"), self.Stand())
