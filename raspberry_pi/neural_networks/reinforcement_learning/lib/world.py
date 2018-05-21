import time
import datetime
import sys
import math
import RPi.GPIO as GPIO # for reset button
from lib.state import State
from lib.base_world import BaseWorld
import pdb
import numpy as np

# Don't try to interface with robot
# peripherals unless you are a robot.
if(sys.platform == 'linux'):
  from lib.arduino import Arduino
else:
  from lib.fakes.arduino_fake import Arduino

class SomeSpace():
  def __init__(self, shape, high=1., low=-1.):
    self.shape = (shape,0)
    self.high = np.array([high])
    self.low = np.array([low])

class World(BaseWorld):
  MAX_EPISODE_DURATION = 10 # seconds
  MAX_ANGLE = 0.25 # in radians. ~> 14.3 degrees
  MAX_DISTANCE = 1 # in meters.

  def __init__(self):
    self.state = State()
    self.arduino = Arduino()
    self.observation_space = SomeSpace(4)
    self.action_space = SomeSpace(1, 1.0, -1.0)
    self.episodeStartTime = datetime.datetime.now()
    self.arduino.resetRobot()

    # Pin Setup:
    self.buttonPin = 4
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

  # returns four states
  def getObservation(self):
    self.observation = self.arduino.getObservation()
    return self.observation

  # returns observation, reward, done
  def step(self, action):
    self.updateMotors(action)
    observation = self.getObservation()
    if observation == False:
      return False, False, False

    return observation, self.__reward(), self.isDone()

  def reset(self):
    print("initiating reset...")
    self.arduino.stopMotors()
    self.arduino.resetRobot() # primarily setting xPos = 0
    print('\a')
    print("waiting on reset button...")
    while 1:
        if GPIO.input(self.buttonPin): # button is released
          a = 1
           # do Nothing
        else: # button is pressed:
          print("button pressed! Here we go!")
          break

    self.episodeStartTime = datetime.datetime.now()
    return self.getObservation()

  def beginEpisode(self):
    self.episodeStartTime = datetime.datetime.now()
    # self.arduino.give_robot_slack() NOT NEEDED. USING BUTTON

  def updateMotors(self, newPower):
    self.arduino.updateMotorPower(newPower)

  # determines if episode is done
  # conditions:
  # 0) Robot has fallen over
  # 1) Robot is too far from center-balanced
  # 2) Episode is over MAX_EPISODE_DURATION
  def isDone(self):
    return self.__isFallen() or self.__isFarAway() or self.__isRetired()

  def __reward(self):
    return 1

  def __stopMotors(self):
    self.arduino.stopMotors()

  # Is the robot angle or x position too ugly?
  def __isFallen(self):
    if(math.fabs(self.observation[2]) > self.MAX_ANGLE):
      print("!!!!!!!!!!!!!!!isFallen. Angle: {} > {}".format(self.observation[2], self.MAX_ANGLE))
      return True
    else:
      return False

  def __isFarAway(self):
    if(math.fabs(self.observation[0]) > self.MAX_DISTANCE):
      print("!!!!!!!!!!!!!!!isFarAway. Distance: {} > {}".format(self.observation[0], self.MAX_DISTANCE))
      return True
    else:
      return False

  # Is the episode over MAX_EPISODE_DURATION
  def __isRetired(self):
    delta = datetime.datetime.now() - self.episodeStartTime
    if(delta.seconds > self.MAX_EPISODE_DURATION):
      print("!!!!!!!!!!!!!!!isRetired {} > {}".format(delta.seconds, self.MAX_EPISODE_DURATION))
      return True
    else:
      return False

