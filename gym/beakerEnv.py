# from pybullet_envs.scene_abstract import SingleRobotEmptyScene
from pybullet_envs.scene_stadium import SinglePlayerStadiumScene
from pybullet_envs.env_bases import MJCFBaseBulletEnv
from beakerBot import BeakerBot
import gym, gym.spaces, gym.utils, gym.utils.seeding
import numpy as np
import pybullet as p
import os, sys
import pdb

class BeakerCam:
	def __init__(self, bullet_client):
		self._p = bullet_client

	def move_and_look_at(self,x,y,z):
		lookat = [x,y,z]
		distance = 1
		yaw = 50
		self._p.resetDebugVisualizerCamera(distance, yaw, -20, lookat)

class BeakerBotBulletEnv(MJCFBaseBulletEnv):
	def __init__(self):
		self.robot = BeakerBot()
		MJCFBaseBulletEnv.__init__(self, self.robot)
		self.stateId=-1

	def create_single_player_scene(self, bullet_client):
		return SinglePlayerStadiumScene(bullet_client, gravity=9.8, timestep=0.0165, frame_skip=1)

	def _reset(self):
		print("reset: gym_pemdulum_envs _reset()")
		# pdb.set_trace()
		if (self.stateId>=0):
			print("I guess stateId >= 0")
			print("InvertedPendulumBulletEnv reset p.restoreState(",self.stateId,")")
			pdb.set_trace()
			self._p.restoreState(self.stateId)

		r = MJCFBaseBulletEnv._reset(self)

		if (self.stateId<0):
			print("I guess stateId < 0")
			self.stateId = self._p.saveState()
			print("InvertedPendulumBulletEnv reset self.stateId=",self.stateId)

		return r

		# print("reset: in beakerEnv")
		# r = MJCFBaseBulletEnv._reset(self) # this is the line that makes the second robot
		# self.beakerCam = BeakerCam(self._p)
		# self.camera_adjust()

		# return r
	
	def _step(self, a):
		self.robot.apply_action(a)
		self.scene.global_step()
		state = self.robot.calc_state()
		vel_penalty = 0
		reward = 1.0
		done = np.abs(self.robot.theta) > .2
		self.rewards = [float(reward)]
		self.HUD(state, a, done)
		return state, sum(self.rewards), done, {}

	def camera_adjust(self):
		self.beakerCam.move_and_look_at(0,0,0)


# class InvertedPendulumBulletEnv(MJCFBaseBulletEnv):
# 	def __init__(self):
# 		self.robot = InvertedPendulum()
# 		MJCFBaseBulletEnv.__init__(self, self.robot)
# 		self.stateId=-1

# 	def create_single_player_scene(self, bullet_client):
# 		return SingleRobotEmptyScene(bullet_client, gravity=9.8, timestep=0.0165, frame_skip=1)

# 	def _reset(self):
# 		print("reset: gym_pemdulum_envs _reset()")
# 		# pdb.set_trace()
# 		if (self.stateId>=0):
# 			print("I guess stateId >= 0")
# 			print("InvertedPendulumBulletEnv reset p.restoreState(",self.stateId,")")
# 			# pdb.set_trace()
# 			self._p.restoreState(self.stateId)
# 		pdb.set_trace()
# 		r = MJCFBaseBulletEnv._reset(self)
# 		if (self.stateId<0):
# 			print("I guess stateId < 0")
# 			self.stateId = self._p.saveState()
# 			print("InvertedPendulumBulletEnv reset self.stateId=",self.stateId)
# 		return r
	
# 	def _step(self, a):
# 		self.robot.apply_action(a)
# 		self.scene.global_step()
# 		state = self.robot.calc_state()  # sets self.pos_x self.pos_y
# 		vel_penalty = 0
# 		if self.robot.swingup:
# 			reward = np.cos(self.robot.theta)
# 			done = False
# 		else:
# 			reward = 1.0
# 			done = np.abs(self.robot.theta) > .2
# 		self.rewards = [float(reward)]
# 		self.HUD(state, a, done)
# 		return state, sum(self.rewards), done, {}

# 	def camera_adjust(self):
# 		self.camera.move_and_look_at(0,1.2,1.0, 0,0,0.5)