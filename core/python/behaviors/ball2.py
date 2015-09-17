import memory, pose, commands, cfgstiff, core, mem_objects, math
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
    def __init__(self):
      self.toTurn = math.pi / 64
      self.toPan = 0
      super(self.__class__, self).__init__()

    def run(self):
      commands.stand()
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        self.finish()
        self.toPan = ball.visionBearing
        commands.setHeadPan(ball.visionBearing, target_time = 0.15)
        print ball.visionElevation
        print ball.elevation
      else:
        print (core.joint_values[core.HeadPan])
        if (core.joint_values[core.HeadPan] >= ((math.pi / 4) - 0.05)):
          self.toTurn = (-1)*(math.pi / 64) 
        elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 4) + 0.05)):
          self.toTurn = (math.pi / 64)
        new_value = self.toPan + self.toTurn
#        print "new value"
#        print new_value
#        print "max cap"
        max_cap = max(new_value, (-1)*(math.pi/4))
 #       print max_cap
 #       print "min cap"
        min_cap = min(max_cap, math.pi / 4)
 #       print min_cap 
        self.toPan = min_cap
        commands.setHeadPan(self.toPan, target_time = .15)

      if self.getTime() > 30.0:
        memory.speech.say("playing stand complete")
        self.finish()

  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  class Look(Node):
    def run(self):
      commands.stand()
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        new_value = ball.visionBearing
        max_cap = max(new_value, (-1)*(math.pi/2) + 0.01)
        min_cap = min(max_cap, (math.pi / 2) - 0.01)
        commands.setHeadPan(min_cap, target_time = 0.35)
      if self.getTime() > 30.0:
        memory.speech.say("playing stand complete")
        self.finish()    

  def setup(self):
    stand = self.Stand()
    walk = self.Walk()
    look = self.Look()
    sit = pose.Sit()
    off = self.Off()
    self.trans(stand, C, look, C, off)
