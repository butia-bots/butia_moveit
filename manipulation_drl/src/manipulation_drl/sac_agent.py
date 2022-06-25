#!/usr/bin/env python3

import rospy
from reach_env_gazebo import DoRISReachEnv
from gym.wrappers.time_limit import TimeLimit
import ray
from ray.rllib.agents.sac import SACTrainer
from ray import tune
from ray.tune import Trainable
from ray.tune.integration.wandb import WandbLoggerCallback, WandbTrainableMixin, wandb_mixin
import wandb
from ray.tune.registry import register_env


def env_creator(env_config):
    env = DoRISReachEnv()
    env = TimeLimit(env, max_episode_steps=50)
    return env


register_env("doris_reach_env", env_creator)

train = True
restore = None
test = False

#run = wandb.init(name='SAC_pick_place', project="Manipulation DRL")

if __name__ == '__main__':
    rospy.init_node('sac_agent')
    #env = DoRISReachEnv()
    #env = TimeLimit(env, max_episode_steps=50)
    config = SACTrainer.get_default_config()
    config['prioritized_replay'] = True
    config['env'] = "doris_reach_env"
    config['horizon'] = 50
    config['num_workers'] = 1
    config['num_envs_per_worker'] = 1
    ray.init(local_mode=True)
    if train == True:
        tune.run(
            "SAC",
            stop={
                "episode_reward_mean": 250,
            },
            restore=restore,
            config=config,
            callbacks=[
                WandbLoggerCallback(project="DoRISReach")
            ],
        )
    if test == True:
        env = DoRISReachEnv()
        env = TimeLimit(env, max_episode_steps=50)
        agent = SACTrainer(config=config, env=env)
        agent.restore(
            "/home/cris/butia_ws/src/butia_moveit/manipulation_drl/agents/sac_rllib_reach")
        for i in range(100):
            obs = env.reset()
            done = False
            while not done:
                action = agent.compute_action(obs)
                obs, reward, done, info = env.step(action)

# run.finish()
