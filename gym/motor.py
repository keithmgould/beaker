# This is a big cheat. I know from testing the real robot that
# change in rotational velocity is about 4 rad/sec per 20 ms.
# So, I'm going to just try to simulate that here and continue
# to use the joint.set_velocity method, rather than useing the
# set_torque, which would require a more complicated internal
# model
#
# Note: both real Beaker and virtual beaker operate at 50hz
#
# trying to avoid this:
# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/minitaur/envs/motor.py

class Motor:
	maxChange = 4

	@classmethod
	def step(cls, currentSpeed, goalSpeed):
		minVal = currentSpeed - cls.maxChange
		maxVal = currentSpeed + cls.maxChange
		return cls.constrain(goalSpeed, minVal, maxVal)

	@classmethod
	def constrain(cls, val, min_val, max_val):
		return min(max_val, max(min_val, val))