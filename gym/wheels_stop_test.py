# This is a test to help ensure the Beaker model matches the real Beaker.
#
# The goal of this test is to ensure the friction in models' axle/wheel match
# the real Beaker's friction.

# The configuration is the wheels are in the air, not touching the ground.
# Once wheels spinning to 10 rads/sec and power switched off, how long 
# does it take until they stop? Once that time is determined on the real
# Beaker, use this test to play with the friction value until the time matches
# for the virtual Beakers wheels.

import gym, register
import time

def main():
	env = gym.make("BeakerBotBulletEnv-v0")
	env.render(mode="human")
	obs = env.reset()
	targetRPS = 20

	while True:
		# BeakerEnv steps at 50Hz. Just use this if rendering
		time.sleep(1./50.) 
		obs, r, done, _ = env.step(targetRPS)
		print(obs[3])

if __name__=="__main__":
	main()