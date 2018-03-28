import tensorflow as tf
from policy import Policy

with tf.Graph().as_default(), tf.Session() as session:
  policy = Policy(session, 4, -0.7, 0.7)
  session.run(tf.global_variables_initializer())
  weight_tensor = tf.get_default_graph().get_tensor_by_name('hidden/kernel:0')
  weights = session.run(weight_tensor)
  print(weights)
