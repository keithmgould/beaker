import gym, register
import numpy as np
import pybullet_envs
import time
import pdb

def main():
    print("create env")
    env = gym.make("InvertedPendulumBulletEnv-v0")
    env.render(mode="human")

    while 1:
        frame = 0
        score = 0
        restart_delay = 0
        print("1) reseting from test client")
        # pdb.set_trace()
        obs = env.reset()
        while 1:
            frame += 1
            time.sleep(1./60.)
            if frame > 100:
                break
   
            obs, r, done, _ = env.step([0])

            still_open = env.render("human")
            if still_open==False:
                return
            if not done: continue
            if restart_delay==0:
                
                restart_delay = 60*2  # 2 sec at 60 fps
            else:
                restart_delay -= 1
                if restart_delay==0: break




if __name__=="__main__":
    main()