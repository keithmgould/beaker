import numpy as np
import cma
import pdb
import gym
import time
import register

# NN hyperparameters
input_size = 4
hidden_size = 16
output_size = 1
# end hyperparameters

env = gym.make("BeakerBotBulletEnv-v0")
env.render(mode="human")

class NeuralNetwork:
  def __init__(self, input_size, hidden_size, output_size):
    self.input_size = input_size
    self.hidden_size = hidden_size
    self.output_size = output_size
    self.w1 = np.zeros((input_size, hidden_size))
    self.b1 = np.zeros(hidden_size)
    self.w2 = np.zeros((hidden_size, output_size))
    self.b2 = np.zeros(output_size)

  def load_weights_and_biases(self, values):
    parts = [self.w1, self.b1, self.w2, self.b2]
    pointer = 0

    # load w1
    value_slice = values[pointer:(pointer + self.w1.size)]
    self.w1 = np.reshape(value_slice, self.w1.shape)
    pointer += self.w1.size

    # load b1
    value_slice = values[pointer:(pointer + self.b1.size)]
    self.b1 = np.reshape(value_slice, self.b1.shape)
    pointer += self.b1.size

    # load w2
    value_slice = values[pointer:(pointer + self.w2.size)]
    self.w2 = np.reshape(value_slice, self.w2.shape)
    pointer += self.w2.size

    # load b2
    value_slice = values[pointer:(pointer + self.b2.size)]
    self.b2 = np.reshape(value_slice, self.b2.shape)
    pointer += self.b2.size

  def forward(self, x):
    post_hidden = np.tanh(np.matmul(x, self.w1) + self.b1)
    return np.tanh(np.matmul(post_hidden, self.w2) + self.b2)

  def neuron_count(self):
    return self.w1.size + self.b1.size + self.w2.size + self.b2.size

model = NeuralNetwork(input_size, hidden_size, output_size)

print("Neuron (parameter) count: {}".format(model.neuron_count()))

starting_sigma = 1
es = cma.CMAEvolutionStrategy(model.neuron_count() * [0], starting_sigma)

def run_episode():
  obs = env.reset()
  done = False
  final_return = 0

  while not done:
    # time.sleep(1./50.) 
    action = model.forward(obs)
    # print(action)
    action *= 10 # scale up for robot actuation
    obs, r, done, _ = env.step(action)
    final_return += r

  return final_return

while not es.stop():
  solutions = es.ask()
  results = []
  for i, solution in enumerate(solutions):
    print("looking at solution {}".format(i))
    model.load_weights_and_biases(solution)
    runs = []
    for j in range(10):
      runs.append(run_episode())
    results.append(sum(runs) / len(runs))

    print("Average of results: {}".format(sum(results)/len(results)))

  es.tell(solutions, results)
es.result_pretty()