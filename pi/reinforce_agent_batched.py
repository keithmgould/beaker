import numpy as np
import pdb, shutil, time
import argparse
from lib.world import World
import tensorflow as tf
from scipy.stats import norm
from tensorflow.python.ops import random_ops
from policy_tb import Policy

# utility for testing loop times
current_milli_time = lambda: int(round(time.time() * 1000))

class Agent:
  def __init__(self, args):
    self.args = args
    self.env = World()

  def print_episode_results(self, ep_index, action_lengths):
    print("Episode {0}. Steps {1}. Avg {2:.2f}".format(
      ep_index,
      action_lengths[-1],
      np.average(action_lengths[-10:])
    ))

  def run(self):
    with tf.Graph().as_default(), tf.Session() as session:
      # policy = Policy(session, self.env.observation_space.shape[0], self.env.action_space.low[0], self.env.action_space.high[0])
      policy = Policy(session, self.env.observation_space.shape[0], -0.7, 0.7)
      session.run(tf.global_variables_initializer())
      writer = tf.summary.FileWriter("./logs/", graph=tf.get_default_graph())
      saver = tf.train.Saver(max_to_keep=5)
      batch = 0
      if self.args['restore'] or self.args['resume']:
        print("Restoring from saved model.")
        saver.restore(session, tf.train.latest_checkpoint('./models/'))
        file = open("./models/at_batch.txt", "r")
        batch = int(file.read())
        file.close()


      while(True):
              print('=====\nBATCH {}\n===='.format(batch))
              batch_observations, batch_actions, batch_rewards = [], [], []
              ep_lengths = []
              global_episode = 0
              for ep_index in range(10):
                observations, actions, rewards = self.policy_rollout(policy)
                batch_observations.extend(observations)
                batch_actions.extend(actions)
                advantages = [len(rewards)] * len(rewards)
                batch_rewards.extend(advantages)
                ep_length = len(actions)
                ep_lengths.append(ep_length)
                global_episode = (ep_index+1)+(10 * batch)
                print('Episode {} steps: {}'.format(global_episode, ep_length))
              batch_rewards = (batch_rewards - np.mean(batch_rewards)) / (np.std(batch_rewards) + 1e-10)
              policy.update_parameters(batch_observations, batch_actions, batch_rewards)
              print("AVG: {0:.2f}".format(np.mean(np.array(ep_lengths))))
              saver.save(
                session,
                './models/my_model',
                global_step=global_episode,
                write_meta_graph=False)
              file = open("./models/at_batch.txt","w")
              batch = batch + 1
              file.write(str(batch))
              file.close

  def policy_rollout(self, policy):
    observation, reward, done = self.env.reset(), 0, False
    observations, actions, rewards  = [], [], []
    throw_away_action = policy.select_action(observation) # prime network.

    step = 0
    last_time = current_milli_time()
    while not done:
      time_delta = current_milli_time() - last_time
      print("Loop Time: {}".format(time_delta))
      last_time = current_milli_time()
      if step == 2:
        self.env.arduino.give_robot_slack()
        time_delta = current_milli_time() - last_time
      action = policy.select_action(observation)
      time_delta = current_milli_time() - last_time
      observations.append(observation)
      actions.append(action)
      print("obsv: {}, action: {}".format(observation, action))
      observation, reward, done = self.env.step(action)
      time_delta = current_milli_time() - last_time
      rewards.append(reward)
      step = step + 1

    return observations, actions, rewards

  def discount_rewards(self, rewards):
    discounted_rewards = np.zeros_like(rewards)
    running_add = 0
    for t in reversed(range(0, len(rewards))):
      running_add = running_add * 0.99 + rewards[t]
      discounted_rewards[t] = running_add
    return discounted_rewards

# End of Agent
#----------------------------------------------------------------
# Beginning of Main

parser = argparse.ArgumentParser(description='provide arguments for agent')
parser.add_argument('--restore', help='restore from saved session', default=False)
parser.add_argument('--resume', help='restore from saved session', default=False)
args = vars(parser.parse_args())

shutil.rmtree("./logs", True)
agent = Agent(args)
agent.run()
