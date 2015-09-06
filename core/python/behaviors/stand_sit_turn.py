import memory, pose, commands, cfgstiff, head
from task import Task
from state_machine import *

class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 5.0:
      memory.speech.say("ready to play")
      self.finish()

class Playing(StateMachine):

  class StandStraight(Node):
    def run(self):
      commands.standStraight()
      if self.getTime() > 5.0:
        memory.speech.say("Look! I'm standing straight!")
        self.finish()
  
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        memory.speech.say("I'm just standing normally.")
        self.finish()

  class TurnHead(Node):
    def run(self):
      commands.setHeadPan(1.57)
      self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    stand_without_finish = self.StandStraight()
    stand = self.Stand()
    sit = pose.Sit()
    off = self.Off()
    self.trans(self.StandStraight(), C, pose.Sit(), C, self.TurnHead(), C, pose.Sit(), C, self.Stand(), C, pose.Sit(), C, off)

