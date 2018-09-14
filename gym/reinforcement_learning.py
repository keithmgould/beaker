import gym, register
from keras.models import load_model
import numpy as np
import time, sys
import pdb

class ExperienceReplay(object):
  def __init__(self, max_memory=100, discount=.9):
    self.max_memory = max_memory
    self.memory = list()
    self.discount = discount
    self.isFilled = False
    self.winners = []

  def remember(self, state, obs_reward, action):
    self.memory.append([state, obs_reward, action[0][0]])
    if len(self.memory) > self.max_memory:
      self.isFilled = True
      del self.memory[0]

  def learn(self, model):
    subset_size = int(0.2 * self.max_memory)
    memory = np.array(self.memory)

    # extract all rewards
    all_rewards = memory[:,1]
    all_rewards = all_rewards.astype(float)

    # average of all rewards
    avg_all_rewards = np.average(all_rewards)
    
    # average of subset
    subset_rewards = all_rewards[-subset_size:]
    avg_subset = np.average(subset_rewards)

    print("avg all: {}. avg subset: {}".format(avg_all_rewards, avg_subset))
    if(avg_subset > avg_all_rewards):
      subset_memory = memory[-subset_size:]
      self.winners.append(subset_memory)
      winner_length = len(self.winners)
      print("storing winner. count now: {}".format(winner_length))
      del(self.memory[:subset_size])
      if(winner_length > 20):
        self.train(model)


  def train(self, model):
    print("training!")
    for idx, winner in enumerate(self.winners):
      print("training on winner {}".format(idx))
      train_data_x = np.vstack(winner[:,0])
      train_data_y = winner[:,2]

      # pdb.set_trace()
      model.fit(
        train_data_x, 
        train_data_y, 
        batch_size=32, 
        shuffle=True,
        epochs=10,
        verbose=1)
    self.winners = []
    modelName = "model_suck_it.h5"
    print("Saving model: {}".format(modelName))
    model.save(modelName)


#------------------------------------------------
# enf of ExperienceReplay

def calculate_reward(obs):
  return 1 - abs(obs[0]) - abs(obs[2])

def main():
  env = gym.make("BeakerBotBulletEnv-v0")
  env.render(mode="human")
  obs = env.reset()

  if(len(sys.argv) != 2):
    print("give me a model file as argument please!")
    exit()

  modelfile = sys.argv[1]
  model = load_model(modelfile)
  max_memory = 500 # ten seconds
  exp_replay = ExperienceReplay(max_memory=max_memory)
  
  while True:
    # BeakerEnv steps at 50Hz. Just use this if rendering
    time.sleep(1./50.) 
    
    foo = np.array(obs)
    bar = foo.reshape((1, -1))
    action = model.predict(bar, batch_size=1)
    
    obs_reward = calculate_reward(obs)
    exp_replay.remember(obs, obs_reward, action)

    if(exp_replay.isFilled):
      exp_replay.learn(model)

    obs, r, done, _ = env.step(action)
    
    if(done):
      obs = env.reset()

if __name__=="__main__":
  main()