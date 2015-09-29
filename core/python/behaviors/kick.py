import memory, pose, commands, cfgstiff, core, mem_objects, math
from task import Task
from memory import *
from state_machine import *

class Playing(StateMachine):

  class Ping(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        print "PRINTING", ball.imageCenterY, ball.imageCenterX
  
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 3.0:
        self.finish()
  
  class OrientBallX(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      print "DISTANCE", ball.visionDistance
      if ball.seen:
        if (ball.visionDistance > 125):
          commands.setWalkVelocity(.25,0,(ball.visionBearing / (math.pi / 2)))
        else:
          commands.setWalkVelocity(0, 0, 0)
          self.postSignal("OrientY")
  
  class OrientBallY(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        y_velocity = 0
        if ball.imageCenterX > 129:
          y_velocity = -0.25
        elif ball.imageCenterX < 123:
          y_velocity = 0.25
        
        if y_velocity != 0:
          commands.setWalkVelocity(0, y_velocity, 0)
        else:
          commands.setWalkVelocity(0,0,0)
          self.postSignal("Kick")

  class OrientBallXY(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        y_velocity = 0
        if ball.imageCenterX > 129:
          y_velocity = -0.1
        elif ball.imageCenterX < 123:
          y_velocity = 0.1

        x_velocity = 0
        if ball.imageCenterY < 206:
          x_velocity = 0.1

        if (x_velocity == 0) and (y_velocity == 0):
          commands.setWalkVelocity(0,0,0)
          self.postSignal("Kick")
        else:
          commands.setWalkVelocity(x_velocity, y_velocity, 0)
      else:
        commands.setWalkVelocity(0,0,0)
        self.postSignal("PreSearchForBall")

  class SearchForBall(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        commands.setWalkVelocity(0,0,0)
        self.postSignal("BallWalk")
      else:
        commands.setWalkVelocity(0,0,0.25)

  class Kick(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        print ball.imageCenterX, ball.imageCenterY, ball.visionDistance
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
        commands.setStiffness(cfgstiff.One)
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.finish()

  class OtherKick(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        print ball.imageCenterX, ball.imageCenterY, ball.visionDistance
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
        commands.setStiffness(cfgstiff.One)
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.postSignal("InGoal")

  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class Ballwalk(Node):
    def __init__(self, x_set, threshold, Kp, Kd, Ki):
      self.error_x = 0
      self.derivative_x = 0
      self.integral_x = 0
      self.x_set = x_set
      self.threshold = threshold
      self.Kp = Kp
      self.Kd = Kd
      self.Ki = Ki
      super(self.__class__, self).__init__()

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        memory.speech.say("ball seen")
        print ball.visionDistance
        new_error_x = max(0, ball.visionDistance * math.cos(ball.visionBearing) - self.x_set)
        self.derivative_x = (new_error_x - self.error_x)
        self.integral_x = self.integral_x + new_error_x 
        self.error_x = new_error_x
        
        velocity_x =  self.Kp * self.error_x + self.Kd * self.derivative_x + self.Ki * self.integral_x
        velocity_x = velocity_x / 1000 
        velocity_x = min(velocity_x, 0.6)
        velocity_x = max(0.23, velocity_x)
        
        commands.setWalkVelocity(velocity_x,0,(ball.visionBearing / (math.pi / 2)))
        if self.error_x <= self.threshold:
          commands.setWalkVelocity(0,0,0)
          self.postSignal("GoalTurn")

  class Goalturn(Node):
    def __init__(self):
      super(self.__class__, self).__init__()
  
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      goal = world_objects.getObjPtr(core.WO_OWN_GOAL)
      error = 20
      print goal.seen, math.fabs(ball.imageCenterX - goal.imageCenterX)
      if goal.seen and (math.fabs(ball.imageCenterX - goal.imageCenterX) <= error):
        commands.setWalkVelocity(0,0,0)
        self.postSignal("OrientX")
      else :
        commands.setWalkVelocity(0,0.4, (ball.visionBearing / (math.pi / 2)))

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()
  
  class PreSearchForBall(Node):
    def run(self):
      if not False:
        self.postSignal("FindBall")
      else:
        memory.speech.say("take ball out of goal")
  
  def setup(self):
  #  
  #  kick = self.Kick()
  #  stand = self.Stand()
  #  goal_turn = self.Goalturn()
  #  find_ball = self.SearchForBall()
  #  ballwalk = self.Ballwalk(120, 20, 0.9, 0.2, 0.00075)
  #  orient = self.OrientBallXY()
  #  orient_x = self.OrientBallX()
  #  orient_y = self.OrientBallY()
  #  sit = pose.Sit()
  #  pre_search_for_ball = self.PreSearchForBall()
    
  #  self.trans(stand, C, pre_search_for_ball)
  #  self.trans(pre_search_for_ball, S("FindBall"), find_ball)
  #  self.trans(find_ball, S("BallWalk"), ballwalk)
  #  self.trans(ballwalk, S("GoalTurn"), goal_turn)
  #  self.trans(goal_turn, S("OrientX"), orient_x)
  #  self.trans(orient_x, S("OrientY"), orient_y)
  #  self.trans(orient_y, S("Kick"), kick)
  #  self.trans(goal_turn, S("Kick"), kick)
  #  self.trans(kick, S("NotInGoal"), ballwalk)
  #  self.trans(kick, S("InGoal"), pre_search_for_ball) 
  #  self.setFinish(None)    

    self.trans(self.Stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
  #  self.trans(self.Stand(), C, self.Ping(), T(30.0), self.Off())
  #  self.trans(self.Stand(), C, self.OrientBallY(), C, self.Kick(), C, pose.Sit(), C, self.Off())
  # self.trans(self.Stand(), C, self.Goalturn(), C, self.Off())
  # self.trans(self.Stand(), C, self.Ballwalk(350), C, self.Goalturn(), C, self.Ballwalk(110), C, self.Kick(), C, self.Off())
  # self.trans(self.Stand(), C, self.Ballwalk(350, 20, 1.1, 0.1, 0.00075), C, self.Off())
  # self.trans(self.Stand(), C, self.SearchForBall(), C, self.Ballwalk(120, 20, 0.9, 0.2, 0.00075), C, self.Goalturn(), C, self.OrientBallXY(), C, self.Kick(), C, pose.Sit(), C, self.Off())
   #self.trans(self.Stand(), C, self.Ballwalk(200), C, self.Stand(), C, self.Kick(), C, pose.Sit(), C, self.Off())
