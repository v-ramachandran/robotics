import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

 class Stand(Node):
    def run(self):
      commands.stand()
      #print "head %f " % core.sensor_values[core.headMiddle]
      if self.getTime() > 5.0:
        memory.speech.say("playing stand complete")
        self.finish()

def setup(self):
    stand = self.Stand()
    sit = pose.Sit()
    self.trans(stand, C, sit)
