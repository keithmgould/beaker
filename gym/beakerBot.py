from pybullet_envs.robot_bases import URDFBasedRobot
import numpy as np
import random
import os
import pdb

class BeakerBot(URDFBasedRobot):
	def __init__(self):
		bP=[0, 0, 0.03]
		bO=[0,0,0,1]
	
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
		r = np.random.randint(-100,100,(3)) / 10000.
		newOrientation=[r[0], r[1], r[2], 1]
		self.body.reset_orientation(newOrientation)

	def getFrictionInfo(self):
		lwJ = self._p.getJointInfo(self.uniqueID, self.leftWheelJoint.jointIndex)
		lwj_dampening = lwJ[6]
		lwj_friction = lwJ[7]

		rwj = self._p.getJointInfo(self.uniqueID, self.rightWheelJoint.jointIndex)
		rwj_dampening = rwj[6]
		rwj_friction = rwj[7]

		lw = self._p.getDynamicsInfo(self.uniqueID, self.leftWheel.bodyPartIndex)
		lw_lateral_friction = lw[1]
		lw_rolling_friction = lw[6]
		lw_spinning_friction = lw[7]

		rw = self._p.getDynamicsInfo(self.uniqueID, self.rightWheel.bodyPartIndex)
		rw_lateral_friction = rw[1]
		rw_rolling_friction = rw[6]
		rw_spinning_friction = rw[7]

		return {
			# joint info
			"lwj_damp": lwj_dampening, 
			"lwj_fric": lwj_friction,
			"rwj_damp": rwj_dampening, 
			"rwj_fric": rwj_friction,

			# link info
			"lw_lat_f": lw_lateral_friction,
			"lw_roll_f": lw_rolling_friction,
			"lw_spin_f": lw_spinning_friction,
			"rw_lat_f": rw_lateral_friction,
			"rw_roll_f": rw_rolling_friction,
			"rw_spin_f": rw_spinning_friction
		}

	def _getXposXvel(self):
		lx, lv = self.leftWheelJoint.get_state()
		rx, rv = self.leftWheelJoint.get_state()
		return (lx + rx)/2, (lv + rv)/2

	def _getThetaAndThetaDot(self):
		body_pose = self.robot_body.pose()
		self.body_rpy = body_pose.rpy()
		roll, pitch, yaw = self.body_rpy
		baseVel = self._p.getBaseVelocity(self.uniqueID)
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


