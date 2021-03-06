class Pid:
  def __init__(self, kP, kI, kD, shouldPrint = False):
    self.shouldPrint = shouldPrint
    self.kP = kP
    self.kI = kI
    self.kD = kD
    self.errorSum = 0
    self.previousError = 0
    self.setPoint = 0

  def reset(self):
    self.errorSum = 0
    self.previousError = 0

  def getControl(self, value):
    error = self.setPoint - value
    self.pTerm = self._getKP(error) 
    self.iTerm = self._getKI(error)
    self.dTerm = self._getKD(error)
    final = self.pTerm + self.iTerm + self.dTerm
    if(self.shouldPrint):
      print("{:0.4f} => {:0.4f}. p: {:0.4f}, i: {:0.4f}, d: {:0.4f}".format(error, final, self.pTerm, self.iTerm, self.dTerm))
    return final

  def getTerms(self):
    return [self.pTerm, self.iTerm, self.dTerm]

  def _getKP(self, error):
    return self.kP * error

  def _getKI(self, error):
    self.errorSum += error
    self.errorSum = self._constrain(self.errorSum, -1, 1)
    return self.kI * self.errorSum

  def _getKD(self, error):
    result = self.kD * (error - self.previousError)
    self.previousError = error
    return result

  def _constrain(self, val, min_val, max_val):
    return min(max_val, max(min_val, val))
