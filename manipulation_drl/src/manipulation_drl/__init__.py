import gym
from ray.tune.registry import register_env

def ray_env_creator(env_name):
     def env_creator(env_config):
          return gym.make(env_name, *env_config)
     return env_creator

gym.envs.register(
     id='DoRISPickAndPlace-v1',
     entry_point='manipulation_drl.pick_and_place_env:DoRISPickAndPlaceEnv',
     max_episode_steps=50,
)

#register_env('DoRISPickAndPlace-v1', ray_env_creator('DoRISPickAndPlace-v1'))


gym.envs.register(
     id='DoRISPickAndPlaceShaped-v1',
     entry_point='manipulation_drl.pick_and_place_env:DoRISPickAndPlaceEnv',
     max_episode_steps=50,
     kwargs=dict(
          reward_type="shaped",
     )
)

gym.envs.register(
     id='DoRISPickAndPlaceDense-v1',
     entry_point='manipulation_drl.pick_and_place_env:DoRISPickAndPlaceEnv',
     max_episode_steps=50,
     kwargs=dict(
          reward_type="dense",
     )
)

gym.envs.register(
     id='DoRISGrasp-v1',
     entry_point='manipulation_drl.grasp_env:DoRISGraspEnv',
     max_episode_steps=50,
)

gym.envs.register(
     id='DoRISGraspShaped-v1',
     entry_point='manipulation_drl.grasp_env:DoRISGraspEnv',
     max_episode_steps=50,
     kwargs=dict(
          reward_type="shaped",
     )
)
