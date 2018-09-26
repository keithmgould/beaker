from pybullet_envs.robot_bases import URDFBasedRobot
from motor import Motor
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
		self.uniqueID = 1 # this is a hacky solution. Better to find the actual ID in case it is not 1

	def apply_action(self, action):
		constrainedAction = Motor.step(self.phiDot,action)
		# self.leftWheelJoint.set_velocity(constrainedAction)
		# self.rightWheelJoint.set_velocity(constrainedAction)
		force = 1000
		self._p.setJointMotorControl2(self.uniqueID,self.leftWheelJoint.jointIndex,self._p.VELOCITY_CONTROL, targetVelocity=constrainedAction, force=force)
		self._p.setJointMotorControl2(self.uniqueID,self.rightWheelJoint.jointIndex,self._p.VELOCITY_CONTROL, targetVelocity=constrainedAction, force=force)

	# rads, rads/sec, rads, rads/sec
	def calc_state(self):
		self.theta, self.thetaDot = self._getThetaAndThetaDot()
		self.phi, self.phiDot = self._getWheelPositionAndVelocity()
		return [self.theta, self.thetaDot, self.phi, self.phiDot]

	def robot_specific_reset(self, bullet_client):
		self.body = self.parts["beaker"]
		self.leftWheel = self.parts["left_wheel"]
		self.rightWheel = self.parts["right_wheel"]
		self.leftWheelJoint = self.jdict["base_to_left_wheel"]
		self.rightWheelJoint = self.jdict["base_to_right_wheel"]
		r = np.random.randint(-100,100,(3)) / 10000.
		newOrientation=[r[0], r[1], r[2], 1]

		# newOrientation = self._p.getQuaternionFromEuler([0.50,0,0])
		self.body.reset_orientation(newOrientation)
		self.drawDebugLines()
		# print(self.getFrictionInfo())

	def drawDebugAxis(self, bodyIndex):
		self._p.addUserDebugLine([0,0,0],[0.1,0,0],[1,0,0], parentObjectUniqueId=self.uniqueID, parentLinkIndex=bodyIndex)
		self._p.addUserDebugLine([0,0,0],[0,0.1,0],[0,1,0],parentObjectUniqueId=self.uniqueID, parentLinkIndex=bodyIndex)
		self._p.addUserDebugLine([0,0,0],[0,0,0.1],[0,0,1],parentObjectUniqueId=self.uniqueID, parentLinkIndex=bodyIndex)

	def drawDebugLines(self):
		self.drawDebugAxis(self.body.bodyPartIndex)
		self.drawDebugAxis(self.leftWheel.bodyPartIndex)
		self.drawDebugAxis(self.rightWheel.bodyPartIndex)

  # (avg across both wheels) rads and rads/sec
	def _getWheelPositionAndVelocity(self):
		lx, lv = self.leftWheelJoint.get_state()
		rx, rv = self.rightWheelJoint.get_state()
		return (lx + rx)/2, (lv + rv)/2

	# rads and rads/sec
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


