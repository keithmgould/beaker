import numpy as np
import time
from lib.world import World
world = World()
while(True):
 print(world.getObservation())
 time.sleep(.1)
