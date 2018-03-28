import tensorflow as tf
from policy import Policy

with tf.Graph().as_default(), tf.Session() as session:
  policy = Policy(session, 4, -0.7, 0.7)
  session.run(tf.global_variables_initializer())

  print(weights)
