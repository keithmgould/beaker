import gym, register
import time

class MiniPid:
	def __init__(self, kP, kI, kD):
		self.kP = kP
		self.kI = kI
		self.kD = kD
		self.errorSum = 0
		self.previousError = 0

	def getControl(self, error):
		return self._getKP(error) + self._getKI(error)	+ self._getKD(error)

	def _getKP(self, error):
		return self.kP * error

	def _getKI(self, error):
		self.errorSum += error
		return self.kI * self.errorSum

	def _getKD(self, error):
		result = self.kD * (error - self.previousError)
		self.previousError = error
		return result

def main():
	env = gym.make("BeakerBotBulletEnv-v0")
	env.render(mode="human")
	obs = env.reset()

	while True:
		# BeakerEnv steps at 50Hz. Just use this if rendering
		time.sleep(1./50.) 
		pid = MiniPid(200,0,0)
		theta = obs[0]
		val = pid.getControl(theta)
		# print("{},{}".format(theta, val))
		obs, r, done, _ = env.step(val)
		
		env.render("human")
		if(done):
			env.reset()

if __name__=="__main__":
	main()