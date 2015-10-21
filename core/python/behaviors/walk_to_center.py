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
      print "standing"
      commands.stand()
      if self.getTime() > 5.0:
        self.finish()

  class Walk(Node):    
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class SidewaysWalk(Node):
    def run(self):
      commands.setWalkVelocity(0,0.5,0)

  class TurningWalk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0.175)

  class InplaceTurningWalk(Node):
    def run(self):
      memory.speech.say("round and round")
      commands.setWalkVelocity(0,0,0.22)

  class WalkToCenter(Node):
    def run(self):
      mypos = mem_objects.world_objects[memory.robot_state.WO_SELF]
      currentpos = geometry.Point2D(mypos.loc.x, mypos.loc.y)
      center = geometry.Point2D(0,0);
      bearing = currentpos.getBearingTo(center, mypos.orientation);
      distance = currentpos.getDistanceTo(center);
      if distance <= 150:
        velocity = 0.0
      elif distance <= 500:
        velocity = 0.2
      else:
        velocity = 0.4
      print "distance, bearing",distance, bearing
      commands.setWalkVelocity(velocity * math.cos(bearing), velocity * math.sin(bearing), 0);
      if velocity == 0.0:
        self.finish()

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
 #   self.trans(self.Stand(), C, self.Walk(), T(5.0), self.Stand(), C, sit, C, off)
    self.trans(self.Stand(), C, self.WalkToCenter(), C, off)
