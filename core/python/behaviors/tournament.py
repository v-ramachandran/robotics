import memory, pose, commands, cfgstiff, core, mem_objects, math
from task import Task
from memory import *
from state_machine import *

class Blocker(Node):
  def __init__(self):
    self.timesUnseen = 0
    self.lastNoisyX = 0
    super(self.__class__, self).__init__()  
  
  def __compute_y_at_origin__(self, point1, point2):
    slope = (point2.y - point1.y)/(point2.x - point1.x)
    intercept = point2.y - (slope * point2.x)
    return intercept

  def __position_between_ball__(self):
    pass
  
  def run(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    if ball.seen:
      self.__position_between_ball__()
      self.timesUnseen = 0
      commands.setHeadPan(ball.visionBearing, 0.2)
      if (ball.distance < 500) and (ball.absVel.x < -50):
        UTdebug.log(15, "Ball is close, blocking!")
        y_at_origin = self.__compute_y_at_origin__(ball.loc, ball.endLoc)
        if y_at_origin >= 120:
          choice = "left"
        elif y_at_origin < -120:
          choice = "right"
        else:
         choice = "center"
        self.postSignal(choice)  
      self.lastNoisyX = ball.endLoc.x
    else:
      self.timesUnseen = self.timesUnseen + 1
    if self.timesUnseen > 10:
      commands.setHeadPan(0,0.1)

class BlockingLeft(StateMachine):
  class PerformLogistics(Node):
    def run(self):
      ball = mem_objects.world_objects[core.WO_BALL]
      print "BLOCKING LEFT", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
      UTdebug.log(15, "Blocking left")
      self.finish()

  def setup(self):
    self.trans(pose.PoseSequence(cfgpose.goalieSimBlockLeft, 0.3), T(3.0), self.PerformLogistics(), C, pose.PoseSequence(cfgpose.sittingPoseV3, 1.0))

class BlockingRight(StateMachine):
  class PerformLogistics(Node):
    def run(self):
      ball = mem_objects.world_objects[core.WO_BALL]
      print "BLOCKING RIGHT", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
      UTdebug.log(15, "Blocking right")
      self.finish()

  def setup(self):
    self.trans(pose.PoseSequence(cfgpose.goalieSimBlockRight, 0.3), T(3.0), self.PerformLogistics(), C, pose.PoseSequence(cfgpose.sittingPoseV3, 1.0))

class BlockingCenter(StateMachine):
  class PerformLogistics(Node):
    def run(self):
      ball = mem_objects.world_objects[core.WO_BALL]
      print "BLOCKING CENTER", ball.bearing, ball.distance, ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y
      UTdebug.log(15, "Blocking center")
      self.finish()
    
  def setup(self):
    self.trans(pose.PoseSequence(cfgpose.goalieSimBlockCenter, 0.3), T(3.0), self.PerformLogistics(), C, pose.PoseSequence(cfgpose.sittingPoseV3, 1.0))

class Set(LoopingStateMachine):
  def setup(self):
    blocker = Blocker()
    self.trans(blocker, S("left"), BlockingLeft(), T(5), blocker)
    self.trans(blocker, S("right"), BlockingRight(), T(5), blocker)
    self.trans(blocker, S("center"), BlockingCenter(), T(5), blocker)

class Playing(StateMachine):

  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 2.0:
        self.finish()
  
  ###############################################################################################

  class SearchForBall(Node):
    def __init__(self):
      self.toTurn = math.pi / 96
      self.toPan = 0
      super(self.__class__, self).__init__()

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        commands.setHeadPan(0, target_time = 0.5)
        self.postSignal("BallWalk")
      else:
        if (core.joint_values[core.HeadPan] >= ((math.pi / 3) - 0.05)):
          self.toTurn = (-1)*(math.pi / 96) 
        elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 3) + 0.05)):
          self.toTurn = (math.pi / 96)
        new_value = self.toPan + self.toTurn
        max_cap = max(new_value, (-1)*(math.pi/3))
        min_cap = min(max_cap, math.pi / 3)
        self.toPan = min_cap
        commands.setHeadPan(self.toPan, target_time = 0.75)

  ###############################################################################################

  class BallWalk(Node):
    def __init__(self, x_set, threshold, Kp, Kd, Ki):
      self.unseen = 0
      self.error_x = 0
      self.derivative_x = 0
      self.integral_x = 0
      self.x_set = x_set
      self.threshold = threshold
      self.Kp = Kp
      self.Kd = Kd
      self.Ki = Ki
      super(self.__class__, self).__init__()
    
    def __reset__(self):
      self.unseen = 0
      self.error_x = 0
      self.derivative_x = 0
      self.integral_x = 0
      commands.setWalkVelocity(0,0,0)
    
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        self.unseen = 0
        new_error_x = max(0, ball.visionDistance * math.cos(ball.visionBearing) - self.x_set)
        self.derivative_x = (new_error_x - self.error_x)
        self.integral_x = self.integral_x + new_error_x 
        self.error_x = new_error_x
        
        velocity_x =  self.Kp * self.error_x + self.Kd * self.derivative_x + self.Ki * self.integral_x
        velocity_x = velocity_x / 1000 
        velocity_x = min(velocity_x, 0.6)
        velocity_x = max(0.28, velocity_x)
        
        commands.setWalkVelocity(velocity_x, 0, (ball.visionBearing / (math.pi / 2)))
        if self.error_x <= self.threshold:
          self.__reset__()
          self.postSignal("SearchForBeacon")
      else:
        self.unseen = self.unseen + 1
        if (self.unseen > 15):
          self.__reset__()
          self.postSignal("SearchForBall")

  ###############################################################################################

  class OrientX(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        if (ball.visionDistance > 117):
          commands.setWalkVelocity(.25,0,(ball.visionBearing / (math.pi / 2)))
        else:
          commands.setWalkVelocity(0, 0, 0)
          self.postSignal("OrientY")
  
  ###############################################################################################

  class OrientY(Node):
    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        y_velocity = 0
        if ball.imageCenterX > 128:
          y_velocity = -0.25
        elif ball.imageCenterX < 124:
          y_velocity = 0.25
        
        if y_velocity != 0:
          commands.setWalkVelocity(0, y_velocity, 0)
        else:
          commands.setWalkVelocity(0,0,0)
          self.postSignal("Kick")
  
  ###############################################################################################

  class SearchForBeacon(Node):
    def __init__(self):
      self.toTurn = math.pi / 48
      self.toPan = 0
      super(self.__class__, self).__init__()

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      beacon_pink_yellow = world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW)
      beacon_yellow_pink = world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)
      beacon_blue_yellow = world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW)
      beacon_yellow_blue = world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE)
      beacon_pink_blue = world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE)
      beacon_blue_pink = world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK)

      if beacon_yellow_pink.seen or beacon_pink_yellow.seen or beacon_yellow_blue.seen or beacon_blue_yellow.seen:
        commands.setHeadPan(0, target_time = 0.5)
        self.postSignal("GoalTurnRight")
      elif beacon_blue_pink.seen or beacon_pink_blue.seen:
        commands.setHeadPan(0, target_time = 0.5)
        self.postSignal("GoalTurnLeft")
      else:
        if (core.joint_values[core.HeadPan] >= ((math.pi / 3) - 0.05)):
          self.toTurn = (-1)*(math.pi / 48) 
        elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 3) + 0.05)):
          self.toTurn = (math.pi / 48)
        new_value = self.toPan + self.toTurn
        max_cap = max(new_value, (-1)*(math.pi/3))
        min_cap = min(max_cap, math.pi / 3)
        self.toPan = min_cap
        commands.setHeadPan(self.toPan, target_time = 0.75)
    
  ###############################################################################################

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
        self.postSignal("InGoal")

  ###############################################################################################

  class GoalTurn(Node):
    def __init__(self, turn_left):
      self.turn_left = turn_left
      self.last_goal_bearing = None
      super(self.__class__, self).__init__()
    
    def run(self):
      goal = world_objects.getObjPtr(core.WO_OWN_GOAL)
      ball = world_objects.getObjPtr(core.WO_BALL)
      
      if goal.seen:
        self.last_goal_bearing = goal.visionBearing
        self.turn_left = (goal.visionBearing < 0)

      if self.last_goal_bearing and (abs(self.last_goal_bearing) <= math.pi/24):
        commands.setWalkVelocity(0,0,0)
        self.postSignal("OrientX")
      else:
        scale = 1 if self.turn_left else -1
        if ball.visionDistance > 220:
          commands.setWalkVelocity(0.3, 0, (ball.visionBearing / (math.pi / 2)))
        else: 
          commands.setWalkVelocity(0, scale*0.4, (ball.visionBearing / (math.pi / 2)))

  def setup(self):
    stand = self.Stand()
    find_ball = self.SearchForBall()
    ball_walk = self.BallWalk(120, 20, 0.9, 0.2, 0.00075)
    find_beacon = self.SearchForBeacon()
    goal_turn_left = self.GoalTurn(turn_left = True)
    goal_turn_right = self.GoalTurn(turn_left = False)    
    orient_x = self.OrientX()
    orient_y = self.OrientY()
    kick = self.Kick()

    self.trans(stand, C, find_ball)

    self.trans(find_ball, S("BallWalk"), ball_walk)

    self.trans(ball_walk, S("SearchForBall"), find_ball)
    self.trans(ball_walk, S("SearchForBeacon"), find_beacon)

    self.trans(find_beacon, S("GoalTurnRight"), goal_turn_right)
    self.trans(find_beacon, S("GoalTurnLeft"), goal_turn_left)

    self.trans(goal_turn_right, S("OrientX"), orient_x)
    self.trans(goal_turn_left, S("OrientX"), orient_x)

    self.trans(orient_x, S("OrientY"), orient_y)
    self.trans(orient_y, S("Kick"), kick)

