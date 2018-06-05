# from myURDFBasedRobot import MyURDFBasedRobot
from pybullet_envs.robot_bases import URDFBasedRobot
import numpy as np
import os
import pdb

class BeakerBot(URDFBasedRobot):
	def __init__(self):
		r = np.random.rand(3) / 100.0
		bP=[0, 0, 0.03]
		bO=[r[0], r[1], r[2], 1]
	
		path = os.path.join( os.path.dirname(__file__),'beaker.urdf')
		URDFBasedRobot.__init__(self, path, 'beaker', action_dim=1, obs_dim=4, basePosition=bP, baseOrientation=bO)

	def apply_action(self, action):
		self.leftWheelJoint.set_velocity(action)
		self.rightWheelJoint.set_velocity(action)

	def calc_state(self):
		self.theta, self.thetaDot = self._getThetaAndThetaDot()
		xPos, xVel = self._getXposXvel()
		return [self.theta, self.thetaDot, xPos, xVel]

	def robot_specific_reset(self, bullet_client):
		self.body = self.parts["beaker"]
		self.leftWheel = self.parts["left_wheel"]
		self.rightWheel = self.parts["right_wheel"]
		self.leftWheelJoint = self.jdict["base_to_left_wheel"]
		self.rightWheelJoint = self.jdict["base_to_right_wheel"]

	def _getXposXvel(self):
		lx, lv = self.leftWheelJoint.get_state()
		rx, rv = self.leftWheelJoint.get_state()
		return (lx + rx)/2, (lv + rv)/2

	def _getThetaAndThetaDot(self):
		body_pose = self.robot_body.pose()
		self.body_rpy = body_pose.rpy()
		roll, pitch, yaw = self.body_rpy
		baseVel = self._p.getBaseVelocity(1) 				# fix me
		translated = self._translateAroundZ(yaw)
		wx, _, _ = np.dot(translated, baseVel[1])
		return roll, wx

	# https://en.wikipedia.org/wiki/Rotation_matrix
	# see "Basic Rotations" around Z
	def _translateAroundZ(self, yaw):
		return np.array(
			[[np.cos(-yaw), -np.sin(-yaw), 0],
			[np.sin(-yaw), np.cos(-yaw), 0],
			[		0,			 0, 1]]
		)


