from beaker_env import BeakerBotBulletEnv
import gym
from gym.envs.registration import registry, make, spec

def register(id,*args,**kvargs):
	if id in registry.env_specs:
		return
	else:
		return gym.envs.registration.register(id,*args,**kvargs)

register(
	id='BeakerBotBulletEnv-v0',
	entry_point='beaker_env:BeakerBotBulletEnv',
	timestep_limit=1000,
	reward_threshold=950.0,
)