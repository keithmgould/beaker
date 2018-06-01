from pybullet_envs.robot_bases import XmlBasedRobot
import numpy as np
import os
import pdb

# I created this because of path issues (and I am lazy). I wanted an easy way to specify the entire path of the URDF file,
# and the URDFBasedRobot class does not allow this. 
class MyURDFBasedRobot(XmlBasedRobot):
	"""
	Base class for URDF .xml based robots.
	"""

	def __init__(self, model_urdf, robot_name, action_dim, obs_dim, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1]):
		XmlBasedRobot.__init__(self, robot_name, action_dim, obs_dim, False)

		self.model_urdf = model_urdf
		self.basePosition = basePosition
		self.baseOrientation = baseOrientation
		self.fixed_base = False

	def reset(self, bullet_client):
		self._p = bullet_client
		self.ordered_joints = []

		self.robot_path = os.path.join(os.path.dirname(__file__), self.model_urdf)
		print(self.robot_path)

		self.urdfID = self._p.loadURDF(
			self.robot_path, 
			basePosition=self.basePosition, 
			baseOrientation=self.baseOrientation, 
			useFixedBase=self.fixed_base
		)

		self.parts, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(self._p, self.urdfID)

		self.robot_specific_reset(self._p)

		s = self.calc_state()
		self.potential = self.calc_potential()

		return s

	def calc_potential(self):
		return 0

class BeakerBot(MyURDFBasedRobot):
	def __init__(self):
		r = np.random.rand(3) / 100.0
		bP=[0, 0, 0]
		bO=[r[0], r[1], r[2], 1]
		# bO=[0, 0.707, 0, 0.707]
		MyURDFBasedRobot.__init__(self, 'beaker.urdf', 'beaker', action_dim=1, obs_dim=4, basePosition=bP, baseOrientation=bO)

	def robot_specific_reset(self, bullet_client):
		self.body = self.parts["beaker"]
		self.leftWheel = self.parts["left_wheel"]
		self.rightWheel = self.parts["right_wheel"]
		self.leftWheelJoint = self.jdict["base_to_left_wheel"]
		self.rightWheelJoint = self.jdict["base_to_right_wheel"]

	def apply_action(self, action):
		self.leftWheelJoint.set_velocity(action)
		self.rightWheelJoint.set_velocity(action)

	def calc_state(self):
		theta, thetaDot = self._getThetaAndThetaDot()
		xPos, xVel = self._getXposXvel()
		return [theta, thetaDot, xPos, xVel]

	def _getXposXvel(self):
		lx, lv = self.leftWheelJoint.get_state()
		rx, rv = self.leftWheelJoint.get_state()
		return (lx + rx)/2, (lv + rv)/2

	def _getThetaAndThetaDot(self):
		body_pose = self.robot_body.pose()
		self.body_rpy = body_pose.rpy()
		roll, pitch, yaw = self.body_rpy # relative to world frame
		baseVel = self._p.getBaseVelocity(self.urdfID)
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


