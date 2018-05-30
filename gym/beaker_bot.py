from pybullet_envs.robot_bases import XmlBasedRobot
import numpy as np
import os
import pdb

class MyURDFBasedRobot(XmlBasedRobot):
	"""
	Base class for URDF .xml based robots.
	"""

	def __init__(self, model_urdf, robot_name, action_dim, obs_dim, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], fixed_base=False, self_collision=False):
		XmlBasedRobot.__init__(self, robot_name, action_dim, obs_dim, self_collision)

		self.model_urdf = model_urdf
		self.basePosition = basePosition
		self.baseOrientation = baseOrientation
		self.fixed_base = fixed_base

	def reset(self, bullet_client):
		self._p = bullet_client
		self.ordered_joints = []

		print(os.path.join(os.path.dirname(__file__), self.model_urdf))

		if self.self_collision:
			self.parts, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(self._p, 
				self._p.loadURDF(os.path.join(pybullet_data.getDataPath(), self.model_urdf),
				basePosition=self.basePosition,
				baseOrientation=self.baseOrientation,
				useFixedBase=self.fixed_base,
				flags=pybullet.URDF_USE_SELF_COLLISION))
		else:
			self.parts, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(self._p,
				self._p.loadURDF(os.path.join(os.path.dirname(__file__), self.model_urdf),
				basePosition=self.basePosition,
				baseOrientation=self.baseOrientation,
				useFixedBase=self.fixed_base))

		self.robot_specific_reset(self._p)

		s = self.calc_state()  # optimization: calc_state() can calculate something in self.* for calc_potential() to use
		self.potential = self.calc_potential()

		return s

	def calc_potential(self):
		return 0

class BeakerBot(MyURDFBasedRobot):
	def __init__(self):
		MyURDFBasedRobot.__init__(self, 'beaker.urdf', 'beaker', action_dim=1, obs_dim=4)

	def robot_specific_reset(self, bullet_client):
		self.body = self.parts["beaker"]
		self.leftWheel = self.parts["left_wheel"]
		self.rightWheel = self.parts["right_wheel"]
		self.leftWheelJoint = self.jdict["base_to_left_wheel"]
		self.rightWheelJoint = self.jdict["base_to_right_wheel"]

	def updateTargetRadsPerSec(self, targetRPS):
		return True


	def apply_action(self, action):
		self.leftWheelJoint.set_motor_torque(action)
		self.rightWheelJoint.set_motor_torque(action)
		return True

	def calc_state(self):
		foo = self._p.getEulerFromQuaternion(self.body.current_orientation())
		return [0,foo[0],foo[1],foo[2]]


