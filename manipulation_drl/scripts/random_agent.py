#!/usr/bin/env python3

import rospy
from reach_env import DoRISReachEnv
from gym.wrappers.time_limit import TimeLimit


if __name__ == '__main__':
    rospy.init_node('random_agent')
    env = DoRISReachEnv()
    env = TimeLimit(env, max_episode_steps=50)
    for i in range(10):
        env.reset()
        done = False
        while not done:
            obs, reward, done, info = env.step(env.action_space.sample())
            print(obs)
            print(reward)