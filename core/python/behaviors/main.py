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
  class Comment(Node):
    def run(self):
      #memory.speech.say("this codebase is broken")
      if self.getTime() > 5.0:
        memory.speech.say("no ripping the code")
        self.finish()

  class Stand(Node):
    def run(self):
      commands.stand()
      print "head %f " % core.sensor_values[core.headMiddle]
      if self.getTime() > 5.0:
        print dir(memory)
      #  print dir(core)
        memory.speech.say("playing stand complete")
        self.finish()

  class Walk(Node):
    def run(self):
      print "My head yaw value is %f!" % core.joint_values[core.HeadYaw]
      print "Sensor %f " % core.sensor_values[core.centerButton] 
      commands.setWalkVelocity(0.5,0,0)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    comment = self.Comment()
    stand = self.Stand()
    walk = self.Walk()
    sit = pose.Sit()
    off = self.Off()
    self.trans(stand, C, comment, C, walk, T(5.0), sit, C, off)
