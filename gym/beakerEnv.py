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
		yaw = 90
		self._p.resetDebugVisualizerCamera(distance, yaw, -20, lookat)

class BeakerBotBulletEnv(MJCFBaseBulletEnv):
	def __init__(self):
		self.robot = BeakerBot()
		MJCFBaseBulletEnv.__init__(self, self.robot)
		self.stateId=-1

	def create_single_player_scene(self, bullet_client):
		# 50 fps, so a step is 20ms or 0.02 seconds
		return SinglePlayerStadiumScene(bullet_client, gravity=9.8, timestep=0.02, frame_skip=1)

	def _reset(self):
		if (self.stateId>=0):
			self._p.restoreState(self.stateId)

		r = MJCFBaseBulletEnv._reset(self)
		self.beakerCam = BeakerCam(self._p)
		# self.camera_adjust()

		if (self.stateId<0):
			self.stateId = self._p.saveState()

		return r
	
	def _step(self, a):
		self.robot.apply_action(a)
		self.scene.global_step()
		state = self.robot.calc_state()
		done = np.abs(self.robot.theta) > .2
		self.HUD(state, a, done)
		self.camera_adjust(self.robot.robot_body.current_position())
		return state, 1.0, done, {}

	def camera_adjust(self, pos):
		self.beakerCam.move_and_look_at(pos[0], pos[1], pos[2])
