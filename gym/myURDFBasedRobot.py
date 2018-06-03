from pybullet_envs.robot_bases import XmlBasedRobot
import numpy as np
import pdb
import os

# I created this because of path issues. I wanted an easy way to specify the entire path of the URDF file,
# and the pyBullet's native URDFBasedRobot class does not allow this. 
#
# I also got rid of a few other switches that I don't need
class MyURDFBasedRobot(XmlBasedRobot):
	def __init__(self, model_urdf, robot_name, action_dim, obs_dim, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1]):
		XmlBasedRobot.__init__(self, robot_name, action_dim, obs_dim, False)

		self.model_urdf = model_urdf
		self.basePosition = basePosition
		self.baseOrientation = baseOrientation

	def reset(self, bullet_client):
		print("reset: MyURDFBasedRobot")
		self._p = bullet_client
		self.ordered_joints = []

		self.robot_path = os.path.join(os.path.dirname(__file__), self.model_urdf)
		print(self.robot_path)

		pdb.set_trace()
		self.urdfID = self._p.loadURDF(
			self.robot_path, 
			basePosition=self.basePosition, 
			baseOrientation=self.baseOrientation, 
			useFixedBase=False
		)

		
		self.parts, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(self._p, self.urdfID)


		self._robot_specific_reset(self._p)

		s = self.calc_state()
		self.potential = self.calc_potential()

		return s

	def calc_potential(self):
		return 0