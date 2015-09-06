import memory, pose, commands, cfgstiff, core, mem_objects, math
from task import Task
from memory import *
from state_machine import *

global angleToTurn
angleToTurn = 0

class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 5.0:
      memory.speech.say("ready to play")
      self.finish()

class Playing(StateMachine):

  class Walk(Node):

    def __init__(self):
      self.times = 0
      super(self.__class__, self).__init__()
    
    def run(self):
      ball = world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
      if ball.seen:
        if ball.visionDistance > 1500:
          self.times = 0
          commands.setWalkVelocity(0.55,0.0,0.033)
          print ball.visionDistance
          print dir(ball)
        else:
          self.times = self.times + 1
          commands.setWalkVelocity(0,0,0)
          if self.times > 3:
            self.finish()
      else:
        commands.setWalkVelocity(0,0,0.5)        

  class TurnInPlace(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
      if ball.seen:
        toTurn = ball.visionBearing / math.pi
        if toTurn > 0.05 or toTurn < -0.05:
          commands.setWalkVelocity(0,0,toTurn)
          print toTurn
          print dir(ball)
        else:
          commands.setWalkVelocity(0,0,0)
          self.finish()
      else:
        self.finish()

  class TurnUntilSpotted(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
      if ball.seen:
        memory.speech.say("i found the goal")
        commands.setWalkVelocity(0,0,0)
        self.finish()
      else:
        commands.setWalkVelocity(0,0,0.25)

  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 3.0:
        memory.speech.say("playing stand complete")
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.TurnUntilSpotted(), C, self.TurnInPlace(), C, pose.Sit(), C, self.Stand(), C, self.Walk(), C, pose.Sit(), C, self.Off())

