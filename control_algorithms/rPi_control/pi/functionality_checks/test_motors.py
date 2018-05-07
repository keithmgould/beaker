import numpy as np
import time
print("Loading world...")
from lib.world import World
print("instantiating world...")
world = World()
world.updateMotors(.20)
time.sleep(1)
world.updateMotors(0)


