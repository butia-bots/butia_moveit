from gc import callbacks

import torch
import manipulation_drl
from manipulation_drl.pick_and_place_env import DoRISPickAndPlaceEnv
from manipulation_drl.pick_and_place_task import DoRISPickAndPlaceTask
from ray.rllib.agents.callbacks import DefaultCallbacks
from ray.rllib.agents import sac
from ray.rllib import *
import ray
import copy
import random

class HerCallback(DefaultCallbacks):
    def on_learn_on_batch(self, *, policy: Policy, train_batch: SampleBatch, result: dict, **kwargs) -> None:
        '''
        Test of Hindsight experience Replay with the optimum goal and a random goal. '''
        HER_OPTIMUM = policy.config['env_config']['HER_OPT']
        if policy.config['env_config']['HER_RAND_GOALS']>0:
            HER_RANDOM = True
        else:
            HER_RANDOM = False
        RAND_GOALS = policy.config['env_config']['HER_RAND_GOALS']
        print(f'Start length: {len(train_batch)}')

        def scaling(input):
            ''' Scaling function, because I scale the observation for better stability. This could also be imported from the environment'''
            return float(input)

        def reward(goal, hit):
            return DoRISPickAndPlaceTask.reward_function(hit, goal, 0.05)

        DESIRED_GOAL_START = -3
        DESIRED_GOAL_END = None
        ACHIEVED_GOAL_START = -6
        ACHIEVED_GOAL_END = -3

        if HER_OPTIMUM:
            train_batch_her_opt = copy.deepcopy(train_batch)
            for i in range(len(train_batch_her_opt['obs'])):
                #train_batch_her_opt['obs'][i][0] = scaling(train_batch_her_opt['obs'][i]['achieved_goal'])
                train_batch_her_opt['obs'][i][DESIRED_GOAL_START:DESIRED_GOAL_END] = train_batch_her_opt['obs'][i][ACHIEVED_GOAL_START:ACHIEVED_GOAL_END]
                #train_batch_her_opt['infos'][i]['diff_to_goal'] = 0.0
                #train_batch_her_opt['new_obs'][i][0] = scaling(train_batch_her_opt['obs'][i]['achieved_goal'])
                train_batch_her_opt['rewards'][i] = reward(train_batch_her_opt['obs'][i][ACHIEVED_GOAL_START:ACHIEVED_GOAL_END], train_batch_her_opt['obs'][i][DESIRED_GOAL_START:DESIRED_GOAL_END])

        if HER_RANDOM:
            FIRST_RAND = True
            for rand in range(RAND_GOALS):
                train_batch_her_rand = copy.deepcopy(train_batch)
                GOAL_IDX = random.choice(range(len(train_batch_her_rand['obs'])))
                RAND_GOAL = train_batch_her_rand['obs'][GOAL_IDX][ACHIEVED_GOAL_START:ACHIEVED_GOAL_END]
                for i in range(len(train_batch_her_rand)):
                    #train_batch_her_rand['obs'][i][0] = scaling(RAND_GOAL)
                    train_batch_her_rand['obs'][i][DESIRED_GOAL_START:DESIRED_GOAL_END] = RAND_GOAL
                    #train_batch_her_rand['obs'][i]['diff_to_goal'] = abs(RAND_GOAL-train_batch_her_rand['infos'][i]['achieved_goal'])
                    #train_batch_her_rand['new_obs'][i][0] = train_batch_her_rand['obs'][i][0]
                    train_batch_her_rand['rewards'][i] = reward(RAND_GOAL, train_batch_her_rand['obs'][i][ACHIEVED_GOAL_START:ACHIEVED_GOAL_END])
                if FIRST_RAND:
                    train_batch_her_rand_comb = copy.deepcopy(train_batch_her_rand)
                    FIRST_RAND = False
                else:
                    pass
                    #train_batch_her_rand_comb = SampleBatch.concat_samples([train_batch_her_rand_comb, train_batch_her_rand])

        if HER_RANDOM and HER_OPTIMUM:
            train_batch = SampleBatch.concat_samples([train_batch, train_batch_her_opt, train_batch_her_rand_comb])
        elif HER_RANDOM:
            train_batch = SampleBatch.concat_samples([train_batch, train_batch_her_rand_comb])
        elif HER_OPTIMUM:
            train_batch = SampleBatch.concat_samples([train_batch, train_batch_her_opt])

if __name__ == '__main__':
    config = sac.DEFAULT_CONFIG.copy()
    config['num_gpus'] = 1
    config['num_workers'] = 4
    #config['env_config']['render_mode'] = 'human'
    config['env_config']['render'] = True
    config['env_config']['HER_RANDOM'] = True
    config['env_config']['HER_OPT'] = True
    config['env_config']['HER_RAND_GOALS'] = 4
    config['env_config']['max_steps'] = 50
    config['env_config']['range_goal'] = 50
    config['callbacks'] = HerCallback
    trainer = sac.SACTrainer(config=config, env='DoRISPickAndPlace-v1')
    for i in range(10000):
        print(trainer.train())