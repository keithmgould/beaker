import gym, register
from keras.models import load_model
import numpy as np
import time
import pdb

def main():
	env = gym.make("BeakerBotBulletEnv-v0")
	env.render(mode="human")
	obs = env.reset()
	model = load_model("keras/model_256_b_32.h5")

	while True:
		# BeakerEnv steps at 50Hz. Just use this if rendering
		time.sleep(1./50.) 
		
		foo = np.array(obs)
		bar = foo.reshape((1, -1))
		action = model.predict(bar, batch_size=1)
		obs, r, done, _ = env.step(action)
		
		if(done):
				# print("-----------------------------------------------------------------------")
				# print("-----------------------------------------------------------------------")
				# print("-----------------------------------------------------------------------")
			obs = env.reset()

if __name__=="__main__":
	main()