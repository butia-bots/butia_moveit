#!/usr/bin/env python3

import rospy
from butia_moveit.manipulation_drl.scripts.reach_env_gazebo import DoRISReachEnv
from gym.wrappers.time_limit import TimeLimit
#from imitation.data import rollout
#from imitation.data.wrappers import RolloutInfoWrapper
import pygame

if __name__ == '__main__':
    rospy.init_node('expert_demonstrations')    
    pygame.joystick.init()
    joy = pygame.joystick.Joystick(0)
    rospy.loginfo('Init joystick!')
    while True:
        rospy.loginfo(joy.get_axis(0))
    '''
    env = DoRISReachEnv()
    env = TimeLimit(env, max_episode_steps=50)
    for i in range(10):
        env.reset()
        done = False
        while not done:
            obs, reward, done, info = env.step(env.action_space.sample())
            print(obs)
            print(reward)'''