import memory, pose, commands, cfgstiff, core
from task import Task
from state_machine import *

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 2.5:
        self.finish()

  class Ballwalk(Node)
    def __init__(self):
      self.error = 0
      self.derivative = 0
      self.integral = 0
      super(self.__class__, self).__init__()

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        memory.speech.say("ball seen")
        new_error = ball.visionDistance - 0.02
        print new_error
        self.derivative = (new_error - self.error)
        self.integral = self.integral + new_error
        self.error = new_error 
        kp = 1;
        kd = 0;
        ki = 0;
        velocity =  kp * self.error + kd * self.derivative + ki * self.integral
        commands.setWalkVelocity(velocity,0,0);
        if velocity == 0:
          self.finish();

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

def setup(self):
    stand = self.Stand()
    self.trans(self.Stand(), C, self.Ballwalk(), C, off)

