import cma
import pdb
import gym
import register



env = gym.make("BeakerBotBulletEnv-v0")
env.render(mode="human")
obs = env.reset()

es = cma.CMAEvolutionStrategy(4 * [0], 1)

while not es.stop():
	solutions = es.ask()
	pdb.set_trace()

es.result_pretty()