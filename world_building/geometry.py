import matplotlib.pyplot as plt
import numpy as np
import math
import pdb

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)

line_floor, = ax.plot([-2,2], [0,0], 'g-')

# vertical
line_body, = ax.plot([0,0], [0,1], 'b-')

# 30 degrees
theta_30 = 0.523599 # 30 degrees in radians

# 60 degrees
theta_60 = 1.0472 # 60 degrees in radians

x = math.cos(theta_60)
y = math.sin(theta_60)
line_body_30, = ax.plot([0,x], [0,y], 'b-')

# make it so
plt.axis('off')

# this is the money line!
plt.gca().set_aspect('equal', adjustable='box')
fig.canvas.draw()
plt.pause(1e-17)

# pause so we can see
pdb.set_trace()

