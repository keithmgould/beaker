import numpy as np
import pdb, shutil, time, datetime
import argparse
import RPi.GPIO as GPIO # for reset button
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

    # Pin Setup:
    self.buttonPin = 4
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

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
      log_name = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H_%M_%S')
      writer = tf.summary.FileWriter("./logs/" + log_name, graph=tf.get_default_graph())
      saver = tf.train.Saver(max_to_keep=5)
      ep_index = 0
      action_lengths = []
      if self.args['restore'] or self.args['resume']:
        print("Restoring from saved model.")
        saver.restore(session, tf.train.latest_checkpoint('./models/'))
        file = open("./models/at_episode.txt", "r")
        ep_index = int(file.read())
        print("Restoring to episode: {}".format(ep_index))
        file.close()
        file = open("./models/avg_length.txt", "r")
        avg_length = float(file.read())
        print("Restoring to avg length: {}".format(avg_length))
        file.close()
        for x in range (0,10):
          action_lengths.append(avg_length)
      while(True):
        observations, actions, rewards = self.policy_rollout(policy)
        if observations == False:
          print("Errored on getObservation. Restarting Episode...")
          continue
        if len(actions) < 5:
          print("Skipping short {}-step rollout".format(len(actions)))
          continue # this was almost certainly a run with robot not ready
        if len(action_lengths) < 10:
          action_lengths.append(len(actions))
          print("Skipping until we have 10 episode lengths saved. {} Saved so far.".format(len(action_lengths)))
          continue
        returns = self.discount_rewards(rewards)
        returns = returns / returns[0]
        relative = len(actions) - np.average(action_lengths[-10:])
        returns = returns * relative
        # returns = (returns - np.mean(returns)) / (np.std(returns) + 1e-10)

        print("waiting on Beaker button to update Params...")
        while 1:
            if GPIO.input(self.buttonPin): # button is released
              a = 1
               # do Nothing
            else: # button is pressed:
              print("button pressed! Updating Params!")
              print('\a')
              break
        summaries = policy.update_parameters(observations, actions, returns)
        action_lengths.append(len(actions))
        avg_length = np.average(action_lengths[-10:])
        self.log_scalar('avg_length', avg_length, ep_index, writer)
        writer.add_summary(summaries, global_step=ep_index)
        writer.flush()
        self.print_episode_results(ep_index, action_lengths)
        ep_index = ep_index + 1
        if ep_index % 5 == 0 and ep_index > 0:
          print("=====> Saving model")
          saver.save(
            session,
            './models/my_model',
            global_step=ep_index,
            write_meta_graph=False)
          file = open("./models/at_episode.txt","w")
          file.write(str(ep_index))
          file.close
          file = open("./models/avg_length.txt","w")
          file.write(str(avg_length))
          file.close


  def policy_rollout(self, policy):
    while True:
      observation, reward, done = self.env.reset(), 0, False
      observations, actions, rewards  = [], [], []
      throw_away_action = policy.select_action(observation) # prime network.

      step = 0
      last_time = current_milli_time()
      while not done:
        time_delta = current_milli_time() - last_time
        print("Loop Time: {}".format(time_delta))
        last_time = current_milli_time()
        action = policy.select_action(observation)
        time_delta = current_milli_time() - last_time
        observations.append(observation)
        actions.append(action)
        print("obsv: {}, action: {}".format(observation, action))
        observation, reward, done = self.env.step(action)
        if observation == False: # some sort of error communicating w Arduino
            done = False # prob not needed (already false) but just to be safe
            break # lets try another rollout
        time_delta = current_milli_time() - last_time
        rewards.append(reward)
        step = step + 1

      if done:
        self.env.updateMotors(0)
        return observations, actions, rewards

  def discount_rewards(self, rewards):
    discounted_rewards = np.zeros_like(rewards)
    running_add = 0
    for t in reversed(range(0, len(rewards))):
      running_add = running_add * 0.99 + rewards[t]
      discounted_rewards[t] = running_add
    return discounted_rewards

  def log_scalar(self, tag, value, step, writer):
    summary = tf.Summary(value=[tf.Summary.Value(tag=tag, simple_value=value)])
    writer.add_summary(summary, step)

# End of Agent
#----------------------------------------------------------------
# Beginning of Main

parser = argparse.ArgumentParser(description='provide arguments for agent')
parser.add_argument('--restore', help='restore from saved session', default=False)
parser.add_argument('--resume', help='restore from saved session', default=False)
args = vars(parser.parse_args())

agent = Agent(args)
agent.run()
