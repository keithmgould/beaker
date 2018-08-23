from lib.arduino import Arduino
from lib.pid import Pid

import time

current_milli_time = lambda: int(round(time.time() * 1000))

class PidClient:
    MOMENTUM_CONSTANT = 0.10

    def __init__(self):
      #self.thetaPid = Pid(1.1500,0.0500,16.0000)
      self.thetaPid = Pid(50, 4, 5.0000)
      self.xPosPid = Pid(5,0.0000,0.0000)
      self.arduino = Arduino()

    # returns four states
    def getObservation(self):
      self.state = self.arduino.getState()
      self.theta = self.state[0]
      self.xPos = self.state[2]
      self.phiDot = self.state[3]
      self.outerDt = self.state[4]
      self.targetRPS = self.state[5]

    def updateRadPerSec(self, newRadPerSec):
      self.arduino.updateMotorPower(newRadPerSec)

    def loop(self):
      self.getObservation()
      self.thetaTerm = self.thetaPid.getControl(self.theta)
      self.xPosTerm = self.xPosPid.getControl(self.xPos)

      # emergency breaks
#      if(abs(self.thetaTerm) > 0.30):
#        self.updateRadPerSec(0)
#        return

      # // If the xPosTerm did its job and got us leaning back towards X=0, 
      # // then stop trying to accelerate away from X=0.
      momentum = self.MOMENTUM_CONSTANT *  abs(self.phiDot)

      if self.xPos > 0 and self.theta < -momentum:
        self.xPosTerm = 0

      if self.xPos < 0 and self.theta > momentum:
        self.xPosTerm = 0

      # note that thetaDotTerm and phiDotTerm are zeroed out.
      newRadPerSec = self.thetaTerm + self.xPosTerm
      newRadPerSec = -newRadPerSec
      newRadPerSec = self._constrain(newRadPerSec, -10, 10)
      print("st:{}. sx:{}. t:{}. x:{} => {}".format(self.theta, self.xPos, self.thetaTerm, self.xPosTerm, newRadPerSec))

      self.updateRadPerSec(newRadPerSec)

    def _constrain(self, val, min_val, max_val):
      return min(max_val, max(min_val, val))


WAIT_TIME = 20 # 1000 / 50 = 20

def main():
  pid_client = PidClient()
  last_time = current_milli_time()
  while(True):
    new_time = current_milli_time()
    delta_time = new_time - last_time
    if delta_time < WAIT_TIME:
      continue

    last_time = new_time
    pid_client.loop()

if __name__ == '__main__':
  main()
