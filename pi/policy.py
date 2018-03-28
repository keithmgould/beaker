import tensorflow as tf
import numpy as np

class Policy:
  def __init__(self, session, observation_size, lowest_action, highest_action):
    self.session = session
    self.lowest_action = lowest_action
    self.highest_action = highest_action
    optimizer = tf.train.AdamOptimizer(.001)
    self.observations = tf.placeholder(tf.float32, shape=[None, observation_size], name="observations")
    self.actions = tf.placeholder(tf.float32, name="actions")
    self.returns = tf.placeholder(tf.float32, name="returns")
    normal = self.build_graph()
    self.action = tf.reshape(normal.sample(),[])
    self.log_probs = normal.log_prob(self.actions)
    loss = -tf.reduce_mean(tf.multiply(self.log_probs, self.returns))
    self.trainMe = optimizer.minimize(loss)

  def build_graph(self):
    hidden = tf.layers.dense(self.observations, 128, tf.nn.relu, name="hidden")
    mu = tf.layers.dense(hidden,1, tf.nn.tanh, name="mu")
    self.mu = tf.reshape(mu,[-1])
    raw_sigma = tf.get_variable("raw_sigma",[32], initializer=tf.zeros_initializer())
    sigma = tf.reduce_sum(raw_sigma)
    self.sigma = tf.exp(sigma)
    return tf.contrib.distributions.Normal(self.mu, self.sigma)

  def select_action(self, observation):
    feed = { self.observations: [observation] }
    mu, sigma = self.session.run([self.mu, self.sigma], feed_dict=feed)
    mu= mu[0]

    action = np.random.normal(mu, sigma)
    return np.clip(action, self.lowest_action, self.highest_action)

  def update_parameters(self, observations, actions, returns, ep_index):
    feed = {
      self.observations: observations,
      self.actions: actions,
      self.returns: returns,
    }

    self.session.run(self.trainMe, feed_dict = feed)
