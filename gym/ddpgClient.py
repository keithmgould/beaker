import gym, register
import time
import pdb

import baselines.ddpg.main as ddpg

args = ddpg.parse_args()
if ddpg.MPI.COMM_WORLD.Get_rank() == 0:
    ddpg.logger.configure()

args['env_id'] = 'BeakerBotBulletEnv-v0'
ddpg.run(**args)
