import gym, register
import time
import pdb

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

def constrain(val, minVal, maxVal):
	return max(min(maxVal, val), minVal)

def main():
	env = gym.make("BeakerBotBulletEnv-v0")
	env.render(mode="human")
	obs = env.reset()
	pid = MiniPid(1.15,0.05,16)
	targetRPS = 0

	while True:
		# BeakerEnv steps at 50Hz. Just use this if rendering
		time.sleep(1./50.) 
		
		theta = obs[0]
		acc = pid.getControl(theta)
		targetRPS += acc
		targetRPS = constrain(targetRPS, -10, 10)
		# print("{}, {}".format(obs, targetRPS))
		print(targetRPS)
		obs, r, done, _ = env.step(targetRPS)
		
		if(done):
			obs = env.reset()
			pid.reset()
			targetRPS = 0

if __name__=="__main__":
	main()