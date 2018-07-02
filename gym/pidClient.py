import gym, register
import time
import pdb
from miniPid import MiniPid


def constrain(val, minVal, maxVal):
	return max(min(maxVal, val), minVal)

def main():
	env = gym.make("BeakerBotBulletEnv-v0")
	env.render(mode="human")
	obs = env.reset()
	thetaPid = MiniPid(1.15,0.05,16)
	# thetaPid = MiniPid(11.5,0.5,160)

	phiPid = MiniPid(0,0,0)
	targetRPS = 0

	while True:
		# BeakerEnv steps at 50Hz. Just use this if rendering
		time.sleep(1./50.) 
		
		theta = obs[0]
		phi = obs[2]
		accFromTheta = thetaPid.getControl(theta)
		thetaTerms = thetaPid.getTerms()
		accFromPhi = phiPid.getControl(phi)

		acc = accFromTheta + accFromPhi

		targetRPS += acc
		targetRPS = constrain(targetRPS, -10, 10)
		print("obs: {} - thetaTerms: {} - acc: {} - new rad/s: {}".format(obs, thetaTerms, acc, targetRPS))
		
		obs, r, done, _ = env.step(targetRPS)

		
		if(done):
			print("-----------------------------------------------------------------------")
			print("-----------------------------------------------------------------------")
			print("-----------------------------------------------------------------------")
			obs = env.reset()
			thetaPid.reset()
			targetRPS = 0

if __name__=="__main__":
	main()