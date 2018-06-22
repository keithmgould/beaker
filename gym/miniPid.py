class MiniPid:
	def __init__(self, kP, kI, kD):
		self.kP = kP
		self.kI = kI
		self.kD = kD
		self.errorSum = 0
		self.previousError = 0

	def reset(self):
		self.errorSum = 0
		self.previousError = 0

	def getControl(self, error):
		pTerm = self._getKP(error) 
		iTerm = self._getKI(error)
		dTerm = self._getKD(error)

		return pTerm + iTerm + dTerm

	def _getKP(self, error):
		return self.kP * error

	def _getKI(self, error):
		self.errorSum += error
		return self.kI * self.errorSum

	def _getKD(self, error):
		result = self.kD * (error - self.previousError)
		self.previousError = error
		return result