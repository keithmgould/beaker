import numpy as np
import time
import sys
import pandas
import keras
from keras.models import load_model
from world_env import WorldEnv

import pdb
from miniPid import MiniPid

def constrain(val, minVal, maxVal):
  return max(min(maxVal, val), minVal)

def main():
  if(len(sys.argv) != 2):
    print("give me a world_model file as argument please!")
    exit()

  world_model_file = sys.argv[1]

  world_model = load_model(world_model_file)
  env = WorldEnv(world_model)

  obs = env.reset()

  thetaPid = MiniPid(11.5,0.5,160)
  phiPid = MiniPid(0.2,0,0)
  targetRPS = 0
  momentum = 0.10

  while True:
    # BeakerEnv steps at 50Hz. Just use this if rendering
    # time.sleep(1./50.) 
    env.render()
    
    theta = obs[0]
    phi = obs[2]
    accFromTheta = thetaPid.getControl(theta)
    thetaTerms = thetaPid.getTerms()
    accFromPhi = phiPid.getControl(phi)

    if phi > 0 and theta < -momentum:
      accFromPhi = 0

    if phi < 0 and theta > momentum:
      accFromPhi = 0

    acc = accFromTheta + accFromPhi
    acc = -acc
    targetRPS += acc

    targetRPS = constrain(targetRPS, -10, 10)
    obs, r, done, _ = env.step([targetRPS])

    if(done):
      obs = env.reset()
      thetaPid.reset()
      phiPid.reset()
      targetRPS = 0

if __name__=="__main__":
  main()