import time
import datetime
from lib.world import World

world = World()

while True:
  observation = world.getObservation()
  print('.')
  if(world.isDone()):
    print("\nDone!\n")
    world.episodeStartTime = datetime.datetime.now()
    world.arduino.resetPhi()
  time.sleep(0.25)
