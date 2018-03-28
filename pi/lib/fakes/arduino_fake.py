import random

class Arduino:
  def updateMotorPower(self, newPower):
    return True

  def stopMotors(self):
    return True

  def getPhiData(self):
    return random.random(), random.random()

  def resetPhi(self):
    return True
