import matplotlib.pyplot as plt
import numpy as np
import math

class Renderer:
  def __init__(self):
    self.body_length = 1
    plt.ion()
    self.fig = plt.figure()
    ax = self.fig.add_subplot(111)
    self.line_floor, = ax.plot([-3,3], [0,0], 'g-')
    self.line_reference, = ax.plot([0,0], [0,self.body_length], 'g:')
    self.line_body, = ax.plot([0,0], [0,self.body_length], 'b-')
    plt.axis('off')

  def render(self, pose):
    theta = pose[0]
    x_pos = pose[2]
    x = math.sin(theta) * self.body_length
    y = math.cos(theta) * self.body_length
    self.line_body.set_xdata([x_pos, x_pos + x])
    self.line_body.set_ydata([0, y])
    self.fig.canvas.draw()
    plt.pause(1e-17)