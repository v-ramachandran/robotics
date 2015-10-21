import memory, pose, commands, cfgstiff, geometry, mem_objects, core, math
from task import Task
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
      if self.getTime() > 2.0:
        self.finish()
  
  class ArriveAtCenterWithScan(Node):
    def __init__(self):
      self.toTurn = math.pi / 32
      self.toPan = 0
      self.threshold = 0
      self.thresholdUnits = 1
      super(self.__class__, self).__init__()
    def run(self):
#  {core.WO_BEACON_YELLOW_BLUE:0, core.WO_BEACON_BLUE_YELLOW:0, core.WO_BEACON_BLUE_PINK:0, core.WO_BEACON_PINK_BLUE:0, core.WO_BEACON_YELLOW_PINK:0, core.WO_BEACON_PINK_YELLOW:0}

      mypos = mem_objects.world_objects[memory.robot_state.WO_SELF]
      currentpos = geometry.Point2D(mypos.loc.x, mypos.loc.y)
      center = geometry.Point2D(0,0)
      bearing = currentpos.getBearingTo(center, mypos.orientation)
      distance = currentpos.getDistanceTo(center)
      print "arrive distance ", distance
      if distance <= 30:
        velocity = 0.0
        bearing = 0
      else:
        velocity = 0.35
      commands.setWalkVelocity(velocity, 0, (bearing / (math.pi / 2)))
      
      if (core.joint_values[core.HeadPan] >= ((math.pi / 6) - 0.05)):
        self.toTurn = (-1)*(math.pi / 32) 
      elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 6) + 0.05)):
        self.toTurn = (math.pi / 32)

      new_value = self.toPan + self.toTurn
      max_cap = max(new_value, (-1)*(math.pi/6))
      min_cap = min(max_cap, math.pi / 6)
      self.toPan = min_cap
 
      if velocity == 0.0:
        commands.setHeadPan(0, target_time = 0.5)
        commands.setWalkVelocity(0,0,0)
        self.postSignal("evaluate")
#      else:
        #commands.setHeadPan(self.toPan, target_time = 0.5)

      if self.getTime() >= 10.0:
        commands.setHeadPan(0, target_time = 0.5)
        commands.setWalkVelocity(0,0,0)
        mypos = mem_objects.world_objects[memory.robot_state.WO_SELF]
        currentpos = geometry.Point2D(mypos.loc.x, mypos.loc.y)
        center = geometry.Point2D(0,0)
        bearing = currentpos.getBearingTo(center, mypos.orientation)
        distance = currentpos.getDistanceTo(center)
        print "arrive distance ",distance
        if distance <= 30:
          self.postSignal("evaluate")
        else:
          self.postSignal("localize")        

  class InplaceTurningWalk(Node):
    def __init__(self):
      self.times_completed = 0
      self.seen = False
      self.max_times = 1
      super(self.__class__, self).__init__()
      
    def run(self):
      if self.getTime() > 25.0:
        commands.setWalkVelocity(0,0,0)
        self.postSignal("localize")
      else:
        commands.setWalkVelocity(0,0,0.22)
#      if self.seen:
#        commands.setWalkVelocity(0,0,0)
#        if self.getTime() > 8.0:
#          self.postSignal("localize")
#      else:
#        beacons_seen = 0
#        beacon1 = mem_objects.world_objects[core.WO_BEACON_YELLOW_BLUE]
#        beacon2 = mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW]
#        beacon3 = mem_objects.world_objects[core.WO_BEACON_YELLOW_PINK]
#        beacon4 = mem_objects.world_objects[core.WO_BEACON_PINK_YELLOW]
#        beacon5 = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK]
#        beacon6 = mem_objects.world_objects[core.WO_BEACON_PINK_BLUE]
#
#        if beacon1.seen:
#          beacons_seen = beacons_seen + 1
#        if beacon2.seen:
#          beacons_seen = beacons_seen + 1
#        if beacon3.seen:
#          beacons_seen = beacons_seen + 1
#        if beacon4.seen:
#          beacons_seen = beacons_seen + 1
#        if beacon5.seen:
#          beacons_seen = beacons_seen + 1
#        if beacon6.seen:
#          beacons_seen = beacons_seen + 1
#        if beacons_seen >= 2:
#          commands.setWalkVelocity(0,0,0)
#          self.seen = True
##          self.postSignal("localize")
#        else:
#          commands.setWalkVelocity(0,0,0.2)
#      if self.times_completed > self.max_times:
#        self.postSignal("finish")
#      if self.getTime() > 25.0:
#        self.times_completed = self.times_completed + 1
#        self.postSignal("center")
  
  class Initialize(Node):
    def run(self):
      core.localizationC.reInit()
      self.finish()
  
  class EvaluateGoalState(Node):
    def __init__(self):
      self.toTurn = math.pi / 64
      self.toPan = 0
      self.threshold = 0
      self.thresholdUnits = 1
      super(self.__class__, self).__init__()

    def run(self):
      if (core.joint_values[core.HeadPan] >= ((math.pi / 2) - 0.05)):
        self.toTurn = (-1)*(math.pi / 64) 
      elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 2) + 0.05)):
        self.toTurn = (math.pi / 64)
      
      beacons_seen = 0
      beacon1 = mem_objects.world_objects[core.WO_BEACON_YELLOW_BLUE]
      beacon2 = mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW]
      beacon3 = mem_objects.world_objects[core.WO_BEACON_YELLOW_PINK]
      beacon4 = mem_objects.world_objects[core.WO_BEACON_PINK_YELLOW]
      beacon5 = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK]
      beacon6 = mem_objects.world_objects[core.WO_BEACON_PINK_BLUE]

      if beacon1.seen:
        beacons_seen = beacons_seen + 1
      if beacon2.seen:
        beacons_seen = beacons_seen + 1
      if beacon3.seen:
        beacons_seen = beacons_seen + 1
      if beacon4.seen:
        beacons_seen = beacons_seen + 1
      if beacon5.seen:
        beacons_seen = beacons_seen + 1
      if beacon6.seen:
        beacons_seen = beacons_seen + 1

      if beacons_seen < 2:
#        if self.getTime() >= self.threshold:
#          self.threshold = self.thresholdUnits + self.threshold
        new_value = self.toPan + self.toTurn
        max_cap = max(new_value, (-1)*(math.pi/64))
        min_cap = min(max_cap, math.pi / 64)
        self.toPan = min_cap
        commands.setHeadPan(self.toPan, target_time = 0.5)
      else:
        mypos = mem_objects.world_objects[memory.robot_state.WO_SELF]
        currentpos = geometry.Point2D(mypos.loc.x, mypos.loc.y)
        center = geometry.Point2D(0,0)
        bearing = currentpos.getBearingTo(center, mypos.orientation)
        distance = currentpos.getDistanceTo(center)
        print "evaluate distance", distance
#        if distance <= 50:
#          self.postSignal("finish")
#        else:
#          self.postSignal("center")


      if self.getTime() > 10.0:
        self.toTurn = math.pi / 8
        self.toPan = 0
        self.threshold = 0
        self.thresholdUnits = 1
        mypos = mem_objects.world_objects[memory.robot_state.WO_SELF]
        currentpos = geometry.Point2D(mypos.loc.x, mypos.loc.y)
        center = geometry.Point2D(0,0)
        bearing = currentpos.getBearingTo(center, mypos.orientation)
        distance = currentpos.getDistanceTo(center)
        memory.speech.say("done scanning")
        commands.setHeadPan(0, target_time = 0.5)
        print "evaluate distance",distance
        if distance <= 50:
          self.postSignal("finish")
        else:
          self.postSignal("center")


  class HeadScan(Node):
    def __init__(self):
      self.toTurn = math.pi / 96
      self.toPan = 0
      self.threshold = 0
      self.thresholdUnits = 1
      super(self.__class__, self).__init__()

    def run(self):
      if (core.joint_values[core.HeadPan] >= ((math.pi / 3) - 0.05)):
        self.toTurn = (-1)*(math.pi / 96) 
      elif (core.joint_values[core.HeadPan] <= (((-1) * math.pi / 3) + 0.05)):
        self.toTurn = (math.pi / 96)
      
#      if self.getTime() >= self.threshold:
#        self.threshold = self.thresholdUnits + self.threshold
      new_value = self.toPan + self.toTurn
      max_cap = max(new_value, (-1)*(math.pi/3))
      min_cap = min(max_cap, math.pi / 3)
      self.toPan = min_cap
      commands.setHeadPan(self.toPan, target_time = 0.75)

      if self.getTime() > 15.0:
        self.toTurn = math.pi / 96
        self.toPan = 0
        self.threshold = 0
        self.thresholdUnits = 1
        mypos = mem_objects.world_objects[memory.robot_state.WO_SELF]
        currentpos = geometry.Point2D(mypos.loc.x, mypos.loc.y)
        center = geometry.Point2D(0,0)
        bearing = currentpos.getBearingTo(center, mypos.orientation)
        distance = currentpos.getDistanceTo(center)
        memory.speech.say("done scanning")
        print "scan distance ",distance
        commands.setHeadPan(0, target_time = 0.5)
        if distance <= 50:
          self.postSignal("evaluate")
        else:
          self.postSignal("center")

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    evaluate = self.EvaluateGoalState()
    stand = self.Stand()
    stand_to_complete = self.Stand()
    headscan = self.HeadScan()
    arrive_at_center = self.ArriveAtCenterWithScan()
    initialize = self.Initialize()
    turn = self.InplaceTurningWalk()
    sit = pose.Sit()
    off = self.Off()

#    self.trans(stand, C, initialize)
#    self.trans(initialize, C, headscan)
#    self.trans(headscan, C, off)

    self.trans(initialize, C, turn)
    self.trans(turn, S("center"), arrive_at_center)
    self.trans(headscan, S("center"), arrive_at_center)
    self.trans(headscan, S("evaluate"), evaluate)
    self.trans(arrive_at_center, S("turn"), turn)
    self.trans(arrive_at_center, S("evaluate"), evaluate)
    self.trans(evaluate, S("center"), arrive_at_center)
    self.trans(evaluate, S("finish"), stand_to_complete)
    self.trans(arrive_at_center, S("localize"), headscan)
    self.trans(turn, S("localize"), headscan)
    self.trans(headscan, S("finish"), stand_to_complete)
    self.trans(turn, S("finish"), stand_to_complete)
    self.trans(stand_to_complete, C, off)

 #   self.trans(self.Stand(), C, self.Walk(), T(5.0), self.Stand(), C, sit, C, off)
 #   self.trans(stand, C, self.Initialize(), C, self.InplaceTurningWalk(), C, arrive_at_center, C, self.Stand(), C, off)
