import gym, register
import time

def main():
	env = gym.make("BeakerBotBulletEnv-v0")
	env.render(mode="human")
	obs = env.reset()

	while True:
		# how much time goes by in a step?

		time.sleep(1./50.) # just use this if rendering
		val = obs[0] * 200
		print(val)
		obs, r, done, _ = env.step(val)
		
		env.render("human")
		if(done):
			env.reset()

if __name__=="__main__":
	main()