import gym
import numpy as np 
import matplotlib.pyplot as plt
import pdb

# discrete action :(
env = gym.make('CartPole-v0')

#vector of means(mu) and standard dev(sigma) for each paramater
state=env.reset()

mu=np.random.uniform(size=state.shape)
sigma=np.random.uniform(low=0.001,size=state.shape)

# uses parameter vector W to choose policy for 1 episode,
# returns reward from that episode
def noisy_evaluation(env,W, render = False):
  reward_sum=0
  state=env.reset()
  t=0
  while True:
    t+=1
    action=int(np.dot(W,state)>0)#use parameters/state to choose action
    state,reward,done,info=env.step(action)
    if render and t%3==0: env.render()
    reward_sum += reward
    if done or t > 205:
      break

  return reward_sum

def init_params(mu,sigma,n):
  """take vector of mus, vector of sigmas, create matrix such that """
  l=mu.shape[0]
  w_matrix=np.zeros((n,l))
  for p in range(l):
      w_matrix[:,p]=np.random.normal(loc=mu[p],scale=sigma[p]+1e-17,size=(n,))
  return w_matrix

running_reward=0
n=40;
p=8;
n_iter=20;

state=env.reset()
i=0
plt.ion()

while i < n_iter:
  #initialize an array of parameter vectors (offspring)
  offspring =init_params(mu,sigma,n)

  reward_sums=np.zeros((n))
  
  # try each offspring
  for k in range(n):
    reward_sums[k]=noisy_evaluation(env,offspring[k,:])

  #sort params/vectors based on total reward of an episode using that policy
  rankings=np.argsort(reward_sums)

  #pick p vectors with highest reward
  top_vectors=offspring[rankings,:]
  top_vectors=top_vectors[-p:,:]

  #fit new gaussian from which to sample policy
  for q in range(top_vectors.shape[1]):
    mu[q]=top_vectors[:,q].mean()
    sigma[q]=top_vectors[:,q].std()
  
  running_reward=0.99*running_reward + 0.01*reward_sums.mean()
  print("#############################################################################")
  print("iteration:{},mean reward:{}, running reward mean:{} \n"
    " reward range:{} to {},".format(
        i, reward_sums.mean(),running_reward,reward_sums.min(),reward_sums.max(),
        ))
  plt.scatter(i,reward_sums.mean())
  plt.scatter(i,running_reward,color='r')
  plt.pause(0.001)
  i+=1

# show off:
noisy_evaluation(env, mu, True)
