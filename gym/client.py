import gym, register
import time

def main():
	env = gym.make("BeakerBotBulletEnv-v0")
	env.render(mode="human")
	env.reset()
	while True:
		time.sleep(1./60.) # just use this if rendering
		obs, r, done, _ = env.step(0)
		print(obs)
		env.render("human")

if __name__=="__main__":
	main()
