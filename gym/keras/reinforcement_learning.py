import gym, register
from keras.models import load_model
import numpy as np
import time
import pdb

class ExperienceReplay(object):
  def __init__(self, max_memory=100, discount=.9):
    self.max_memory = max_memory
    self.memory = list()
    self.discount = discount

  def remember(self, state, action):
    self.memory.append([state, action])
    if len(self.memory) > self.max_memory:
      del self.memory[0]

  def get_batch(self, model, batch_size=10):
    len_memory = len(self.memory)
    num_actions = 1
    env_dim = 4
    inputs = np.zeros((min(len_memory, batch_size), env_dim))
    targets = np.zeros((inputs.shape[0], num_actions))
    for i, idx in enumerate(np.random.randint(0, len_memory, size=inputs.shape[0])):
      state_t, action_t, reward_t, state_tp1 = self.memory[idx][0]
      game_over = self.memory[idx][1]

      inputs[i:i+1] = state_t
      # There should be no target values for actions not taken.
      # Thou shalt not correct actions not taken #deep
      targets[i] = model.predict(state_t)[0]
      Q_sa = np.max(model.predict(state_tp1)[0])
      if game_over:  # if game_over is True
        targets[i, action_t] = reward_t
      else:
        # reward_t + gamma * max_a' Q(s', a')
        targets[i, action_t] = reward_t + self.discount * Q_sa
    return inputs, targets

def main():
  env = gym.make("BeakerBotBulletEnv-v0")
  env.render(mode="human")
  obs = env.reset()
  model = load_model("keras/model_256_b_32.h5")
  x_history = 0 # running sum of x offsets, which we are trying to minimize
  loop_memory = 200 # remember last 200 loops

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