import numpy as np
import time
import sys
import pandas
import keras
from keras.models import load_model
import pdb
from world_env import WorldEnv

if(len(sys.argv) != 2):
  print("give me a world_model file as argument please!")
  exit()

world_model_file = sys.argv[1]

world_model = load_model(world_model_file)
env = WorldEnv(world_model)

pose = env.reset()

for i in range(0,1000):
	env.render()
	s2, r, terminal, info = env.step([0])
	print("{} -- {}".format(s2, terminal))
	if(terminal):
		env.reset()