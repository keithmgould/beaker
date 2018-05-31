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
		bP=[0, 0, 0.02]
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

	# theta ?
	# thetaDot ?
	# xPos (avg of both wheels)
	# xVel (avg of both wheels)
	def calc_state(self):
		# pdb.set_trace()
		foo = self._p.getEulerFromQuaternion(self.body.current_orientation())
		bar = self._p.getBaseVelocity(self.urdfID) # this is not correct
		self.theta = foo[0]
		self.thetaDot = bar[1][0]
		return [self.theta, self.thetaDot, "x", bar[1][1], bar[1][1]]


