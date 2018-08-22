from lib.arduino import Arduino
from lib.pid import Pid

class PidClient:
    MOMENTUM_CONSTANT = 0.10

    def __init__():
      self.thetaPid = Pid()
      self.xPosPid = Pid()
      self.arduino = Arduino()

    # returns four states
    def getObservation(self):
      self.state = self.arduino.getState()
      self.theta = self.state[0]
      self.thetaDot = self.state[1]
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

      # // If the xPosTerm did its job and got us leaning back towards X=0, 
      # // then stop trying to accelerate away from X=0.
      momentum = self.MOMENTUM_CONSTANT *  abs(self.phiDot)

      if self.xPos > 0 and self.theta < -momentum:
        self.xPosTerm = 0

      if self.xPos < 0 and self.theta > momentum:
        self.xPosTerm = 0

      # note that thetaDotTerm and phiDotTerm are zeroed out.
      radPerSecDelta = thetaTerm + xPosTerm

      radPerSecDelta = -radPerSecDelta
      newRadPerSec = self.targetRPS
      newRadPerSec += radPerSecDelta
      newRadPerSec = self._constrain(newRadPerSec, -10, 10)

      self.updateRadPerSec(newRadPerSec)

    def _constrain(self, val, min_val, max_val):
      return min(max_val, max(min_val, val))


def main(args):
  pid_client = PidClient()
  pid_client.loop()

if __name__ == '__main__':
  main(args)