import memory, pose, commands, cfgstiff, core, mem_objects, math
from task import Task
from memory import *
from state_machine import *

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 3.0:
        self.finish()

  class Kick(Node):
    def run(self):
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
        commands.setStiffness(cfgstiff.One)
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.finish()

  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class Ballwalk(Node):
    def __init__(self):
      self.error_x = 0
      self.derivative_x = 0
      self.integral_x = 0
      self.error_y = 0
      self.derivative_y = 0
      self.integral_y = 0
      super(self.__class__, self).__init__()

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        memory.speech.say("ball seen")
        print ball.visionDistance
        new_error_x = max(0, ball.visionDistance * math.cos(ball.visionBearing) - 100)
        new_error_y = ball.visionDistance * math.sin(ball.visionBearing)
        if new_error_y < 0:
          new_error_y = min(new_error_y + 30, 0)
        else:
          new_error_y = max(new_error_y - 30, 0)
        
        self.derivative_x = (new_error_x - self.error_x)
        self.derivative_y = (new_error_y - self.error_y)
        self.integral_x = self.integral_x + new_error_x
        self.integral_y = self.integral_y + new_error_y
        self.error_x = new_error_x  
        self.error_y = new_error_y     
        kp = 0.0005
        kd = 0
        ki = 0
        velocity_x =  kp * self.error_x + kd * self.derivative_x + ki * self.integral_x
        velocity_y =  kp * self.error_y + kd * self.derivative_y + ki * self.integral_y
        
        print self.error_x, self.error_y
        commands.setWalkVelocity(velocity_x,(-1)*velocity_y,0)
        if velocity_x <= 0.05:
          commands.setWalkVelocity(0,0,0)
          self.finish();


  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Ballwalk(), C, self.Off())
