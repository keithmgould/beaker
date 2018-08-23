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

    def parseState(self, state):
      self.state = state
      self.theta = state[0]
      self.thetaDot = state[1]
      self.xPos = state[2]
      self.phiDot = state[3]
      self.outerDt = state[4]
      self.targetRPS = state[5]


    def updateRadPerSec(self, newRadPerSec):
      self.arduino.updateMotorPower(newRadPerSec)

    def determineControl(self, state):
      self.parseState(state)
      self.thetaTerm = self.thetaPid.getControl(self.theta)
      self.xPosTerm = self.xPosPid.getControl(self.xPos)

      # emergency breaks
      if(abs(self.thetaTerm) > 0.30):
        return 0


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
      return newRadPerSec


    def _constrain(self, val, min_val, max_val):
      return min(max_val, max(min_val, val))

def main():
  arduino = Arduino()
  pid_client = PidClient()

  while(True):
    state = arduino.waitForState()
    newControl = pid_client.determineControl(state)
    arduino.updateMotorPower(newControl)

if __name__ == '__main__':
  main()
