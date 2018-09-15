import numpy as np
import random
import pdb
import keras
from renderer import Renderer

class WorldEnv:
  def __init__(self, model):
    self.model = model
    self.steps = 0
    self.renderer = Renderer()

  def render(self):
    self.renderer.render(self.pose)

  def reset(self):
    theta = random.uniform(-0.05, 0.05)
    self.pose = [theta,0,0,0]
    self.steps = 0
    return self.pose

  def step(self, action):
    self.steps += 1
    self._calculate_next_pose(action)
    return self.pose, self._reward(), self._done(), 0

  def _calculate_next_pose(self, action):
    state = np.concatenate([self.pose, action])
    state = state.reshape((1, -1))
    self.pose = self.model.predict(state)[0]

  def _reward(self):
    return 1

  def _done(self):
    theta = self.pose[0]
    return self.steps > 200 or abs(theta) > 0.25