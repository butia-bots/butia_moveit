#!/usr/bin/env python3

from reach_env_gazebo import DoRISReachEnv
from gym.wrappers.time_limit import TimeLimit
import time
import rospy
import numpy as np

if __name__ == '__main__':
    rospy.init_node('random_agent')
    #env = PandaPickAndPlaceEnv(render=False)
    env = DoRISReachEnv()
    env = TimeLimit(env, max_episode_steps=50)
    #rate = rospy.Rate(1)
    for i in range(10):
        env.reset()
        #input()
        done = False
        while not done:
            before = time.time()
            obs, reward, done, info = env.step(np.array([1.0, 0.0, 1e-2, 0.0, 1.0, 0.0, 0.0]))
            print(obs)
            print(reward)
            print(1/(time.time() - before))
            time.sleep(1/60.0)
