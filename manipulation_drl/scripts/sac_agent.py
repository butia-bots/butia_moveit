#!/usr/bin/env python3

from mim import train
import rospy
from reach_env import DoRISReachEnv
from gym.wrappers.time_limit import TimeLimit
from stable_baselines3.sac.sac import SAC
import torch

torch.set_num_threads(1)


if __name__ == '__main__':
    rospy.init_node('random_agent')
    env = DoRISReachEnv()
    env = TimeLimit(env, max_episode_steps=50)
    agent = SAC('MlpPolicy', env, verbose=1)
    agent.learn(1000)
    agent.save('/home/cris/butia_env/src/butia_moveit/manipulation_drl/agents/sac_reach/model.zip')
    agent.save_replay_buffer('/home/cris/butia_env/src/butia_moveit/manipulation_drl/agents/sac_reach/replay_buffer.pkl')
