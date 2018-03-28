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
    tf.summary.scalar('loss', loss)
    self.tensorboard_scalar_store(self.actions, "actions")
    self.tensorboard_scalar_store(self.log_probs, "log_probs")

    self.trainMe = optimizer.minimize(loss)
    self.summaries = tf.summary.merge_all()

  def tensorboard_scalar_store(self, thing, family):
    tf.summary.scalar(family + 'max', tf.reduce_max(thing))
    tf.summary.scalar(family + 'min', tf.reduce_min(thing))
    tf.summary.scalar(family + 'mean', tf.reduce_mean(thing))

  def tensorboard_grad_store(self, gvs):
    hidden_weight_grads = gvs[0][0]
    hidden_bias_grads = gvs[1][0]
    mu_weight_grads = gvs[2][0]
    mu_bias_grads = gvs[3][0]
    tf.summary.histogram(family + "weightGrads", hidden_weight_grads)
    tf.summary.histogram(family + "biasGrads", hidden_bias_grads)
    tf.summary.histogram(family + "weightGrads", mu_weight_grads)
    tf.summary.histogram( family + "biasGrads", mu_weight_grads)

  def tensorboard_wba_store(self, family, layer):
    weights = tf.get_default_graph().get_tensor_by_name(family + '/kernel:0')
    bias = tf.get_default_graph().get_tensor_by_name(family + '/bias:0')
    tf.summary.histogram( family + "weights", weights)
    tf.summary.histogram( family + "bias", bias)
    tf.summary.histogram( family + "activations", layer)

  def build_graph(self):
    hidden = tf.layers.dense(self.observations, 128, tf.nn.relu, name="hidden")
    self.tensorboard_wba_store("hidden", hidden)
    mu = tf.layers.dense(hidden,1, tf.nn.tanh, name="mu")
    self.tensorboard_wba_store("mu", mu)
    self.mu = tf.reshape(mu,[-1])
    raw_sigma = tf.get_variable("raw_sigma",[32], initializer=tf.zeros_initializer())
    sigma = tf.reduce_sum(raw_sigma)
    self.sigma = tf.exp(sigma)
    tf.summary.scalar('sigma', self.sigma)
    return tf.contrib.distributions.Normal(self.mu, self.sigma)

  def select_action(self, observation):
    feed = { self.observations: [observation] }
    mu, sigma = self.session.run([self.mu, self.sigma], feed_dict=feed)
    mu= mu[0]

    action = np.random.normal(mu, sigma)
    return np.clip(action, self.lowest_action, self.highest_action)

  def update_parameters(self, observations, actions, returns):
    feed = {
      self.observations: observations,
      self.actions: actions,
      self.returns: returns,
    }

    summaries = self.session.run(self.summaries, feed_dict = feed)
    self.session.run(self.trainMe, feed_dict = feed)
    return summaries

