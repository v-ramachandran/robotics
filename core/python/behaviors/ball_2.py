import memory, pose, commands, cfgstiff, core, mem_objects
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
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        memory.speech.say("ball detected")
        commands.setHeadPan(ball.visionBearing, target_time = 0.5)
        print ball.visionElevation
        print ball.elevation
      else:
        memory.speech.say("there is no ball in sight")
        commands.setHeadPan(0, isChange = True)
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

  def setup(self):
    stand = self.Stand()
    walk = self.Walk()
    sit = pose.Sit()
    off = self.Off()
    self.trans(stand, C, off)

