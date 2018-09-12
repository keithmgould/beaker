from pybullet_envs.scene_stadium import SinglePlayerStadiumScene
from pybullet_envs.env_bases import MJCFBaseBulletEnv
from beakerBot import BeakerBot
import gym, gym.spaces, gym.utils, gym.utils.seeding
import random, math
import numpy as np
import pybullet as p
import os, sys
import pdb

THROWBALLS = False

class BeakerCam:
	def __init__(self, bullet_client):
		self._p = bullet_client

	def move_and_look_at(self,x,y,z):
		lookat = [x,y,z]
		distance = 1
		yaw = 135
		self._p.resetDebugVisualizerCamera(distance, yaw, -20, lookat)

class BeakerBotBulletEnv(MJCFBaseBulletEnv):
	def __init__(self):
		self.robot = BeakerBot()
		MJCFBaseBulletEnv.__init__(self, self.robot)
		self.stateId=-1

	def create_single_player_scene(self, bullet_client):
		# 50 hz, so a step is 20ms or 0.02 seconds
		return SinglePlayerStadiumScene(bullet_client, gravity=9.8, timestep=0.02, frame_skip=1)

	def _reset(self):
		if (self.stateId>=0):
			self._p.restoreState(self.stateId)

		rr = MJCFBaseBulletEnv._reset(self)
		self.beakerCam = BeakerCam(self._p)
		
		if(THROWBALLS and self.stateId < 0):
			self.frames_since_ball_thrown = 0
			mass = .04
			visualShapeId = -1
			sphereRadius = 0.05
			useMaximalCoordinates = 0
			position = [0.3,0.3,0.2]
			colSphereId = self._p.createCollisionShape(p.GEOM_SPHERE,radius=sphereRadius)
			self.sphereUid = p.createMultiBody(mass,colSphereId,visualShapeId,position,useMaximalCoordinates=useMaximalCoordinates)
			self._throw_ball()

		if (self.stateId<0):
			self.stateId = self._p.saveState()

		return rr
	
	def _throw_ball(self):
		random_angle = random.randint(0,360)
		xPos = 1 * math.cos(random_angle)
		yPos = 1 * math.sin(random_angle)
		p.resetBasePositionAndOrientation(self.sphereUid, [xPos,yPos,0.2], [0,0,0,1])
		p.resetBaseVelocity(self.sphereUid, [-xPos * 2,-yPos * 2,3])

	def _step(self, a):
		self.robot.apply_action(a)
		self.scene.global_step()
		state = self.robot.calc_state() # theta, thetaDot, phi, phiDot (rad, rad/sec, rad, rad/sec)
		done = np.abs(self.robot.theta) > .2
		self.HUD(state, a, done)
		self.camera_adjust(self.robot.robot_body.current_position())
		if(THROWBALLS):
			self.frames_since_ball_thrown += 1
			if(self.frames_since_ball_thrown > 125):
				self.frames_since_ball_thrown = 0
				self._throw_ball()

		return state, 1.0, done, {}

	def camera_adjust(self, pos):
		self.beakerCam.move_and_look_at(pos[0], pos[1], pos[2])
