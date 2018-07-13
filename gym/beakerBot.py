from pybullet_envs.robot_bases import URDFBasedRobot
from motor import Motor
import numpy as np
import random
import os
import pdb

class BeakerBot(URDFBasedRobot):
	def __init__(self):
		# bP=[0, 0, 0.03]
		bP=[0, 0, 0.6]
		bO=[0,0,0,1]
	
		path = os.path.join( os.path.dirname(__file__),'beaker.urdf')
		URDFBasedRobot.__init__(self, path, 'beaker', action_dim=1, obs_dim=4, basePosition=bP, baseOrientation=bO)

	def apply_action(self, action):
		constrainedAction = Motor.step(self.phiDot,action)

		constrainedAction = 1

		force = 1000
		self._p.setJointMotorControl2(self.uniqueID,self.leftGearJoint.jointIndex,self._p.VELOCITY_CONTROL, targetVelocity=constrainedAction, force=force)
		self._p.setJointMotorControl2(self.uniqueID,self.rightGearJoint.jointIndex,self._p.VELOCITY_CONTROL, targetVelocity=constrainedAction, force=force)


		
		# self.leftGearJoint.set_velocity(constrainedAction)
		# self.rightGearJoint.set_velocity(constrainedAction)


	# rads, rads/sec, rads, rads/sec
	def calc_state(self):
		self.theta, self.thetaDot = self._getThetaAndThetaDot()
		self.phi, self.phiDot = self._getWheelPositionAndVelocity()

		lx, lv = self.leftWheelJoint.get_state()
		rx, rv = self.rightWheelJoint.get_state()
		print("leftWheelJoint: {}, rightWheelJoint: {}".format(lx, rx))

		return [self.theta, self.thetaDot, self.phi, self.phiDot]

	def robot_specific_reset(self, bullet_client):
		self.body = self.parts["beaker"]
		self.leftGear = self.parts["left_gear"]
		self.rightGear = self.parts["right_gear"]
		self.leftWheel = self.parts["left_wheel"]
		self.rightWheel = self.parts["right_wheel"]
		self.leftGearJoint = self.jdict["body_to_left_gear"]
		self.rightGearJoint = self.jdict["body_to_right_gear"]
		self.leftWheelJoint = self.jdict["left_gear_to_left_wheel"]
		self.rightWheelJoint = self.jdict["right_gear_to_right_wheel"]
		r = np.random.randint(-100,100,(3)) / 10000.
		newOrientation=[r[0], r[1], r[2], 1]

		# newOrientation = self._p.getQuaternionFromEuler([0.50,0,0])
		self.body.reset_orientation(newOrientation)
		self.drawAllDebugAxis()

		# Needed because the revolute joint is "active" and wont spin freely otherwise:
		self._p.setJointMotorControl2(bodyUniqueId=self.uniqueID, jointIndex=self.leftWheelJoint.jointIndex, controlMode=self._p.VELOCITY_CONTROL, force = 0)
		self._p.setJointMotorControl2(bodyUniqueId=self.uniqueID, jointIndex=self.rightWheelJoint.jointIndex, controlMode=self._p.VELOCITY_CONTROL, force = 0)

		# Lets look at the constraints!
		numConstraints = self._p.getNumConstraints()
		print("Number of constraints: {}".format(numConstraints))

		# Lets look at the joint info!
		lwj = self._p.getJointInfo(self.uniqueID, self.leftWheelJoint.jointIndex)
		print("LWJ: {}".format(lwj))
		rwj = self._p.getJointInfo(self.uniqueID, self.rightWheelJoint.jointIndex)
		print("RWJ: {}".format(rwj))

	def drawDebugAxis(self, bodyIndex):
		length = 1
		self._p.addUserDebugLine([0,0,0],[length,0,0],[1,0,0], parentObjectUniqueId=self.uniqueID, parentLinkIndex=bodyIndex)
		self._p.addUserDebugLine([0,0,0],[0,length,0],[0,1,0],parentObjectUniqueId=self.uniqueID, parentLinkIndex=bodyIndex)
		self._p.addUserDebugLine([0,0,0],[0,0,length],[0,0,1],parentObjectUniqueId=self.uniqueID, parentLinkIndex=bodyIndex)

	def drawAllDebugAxis(self):
		self.drawDebugAxis(self.body.bodyPartIndex)
		# left side axis
		self.drawDebugAxis(self.rightGear.bodyPartIndex)
		self.drawDebugAxis(self.rightWheel.bodyPartIndex)
		# right side axis
		self.drawDebugAxis(self.leftGear.bodyPartIndex)
		self.drawDebugAxis(self.leftWheel.bodyPartIndex)

	def getFrictionInfo(self):
		lwJ = self._p.getJointInfo(self.uniqueID, self.leftGearJoint.jointIndex)
		lwj_dampening = lwJ[6]
		lwj_friction = lwJ[7]

		rwj = self._p.getJointInfo(self.uniqueID, self.rightGearJoint.jointIndex)
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

  # (avg across both wheels) rads and rads/sec
	def _getWheelPositionAndVelocity(self):
		lx, lv = self.leftGearJoint.get_state()
		rx, rv = self.leftGearJoint.get_state()
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


