import gym, register
import time, re
import pdb
import numpy as np

env = gym.make("BeakerBotBulletEnv-v0")
env.render(mode="human")

mu = [1.74450761,1.43836396,0.21845779,0.1801307]

def noisy_evaluation(env,W, render = False):
  reward_sum=0
  state=env.reset()
  t=0
  while True:
    t+=1
    action=np.dot(W,state) #use parameters/state to choose action
    state,reward,done,info=env.step(action * 10.0)
    if render:
      env.render()
      time.sleep(1./50.)
    reward_sum += reward
    if done or t > 205:
      break

  return reward_sum



# show off:
noisy_evaluation(env, mu, True)
