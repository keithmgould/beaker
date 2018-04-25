class State:
  def __init__(self):
    self.theta = 0
    self.thetaDot = 0
    self.phi = 0
    self.phiDot = 0

  def all(self):
    return [self.theta, self.thetaDot, self.phi, self.phiDot]
