import pybullet as p
import pybullet_data
import time
import pdb
from beakerBot import BeakerBot

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robot = BeakerBot()
robot.reset(p)

while True:
		p.stepSimulation()
		time.sleep(.001)

p.disconnect()