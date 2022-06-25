from random import sample
import gym

from panda_gym.envs.core import RobotTaskEnv
from panda_gym.pybullet import PyBullet
from panda_gym.utils import distance
from manipulation_drl.doris_robot import DoRISRobot
from gym.spaces import Box
import numpy as np
import pybullet as p
import math

class DoRISGraspEnv(gym.Env):
    def __init__(self, render: bool = False, reward_type: str = "sparse"):
        super().__init__()
        self.sim = PyBullet(render=render)
        self.robot = DoRISRobot(self.sim)
        self.action_space = self.robot.action_space
        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(self.robot.observation_space.shape[0]+6,))
        self.object_range_xy = 0.3
        self.object_size = 0.05
        self.distance_threshold = 0.05
        self.previous_ee_position = None
        self.previous_object_position = None
        self.create_scene()
        
    def render(self, mode):
        if mode == "human":
            self.sim.render()
    
    def step(self, action):
        self.previous_ee_position = self.robot.get_ee_position()
        self.previous_object_position = self.sim.get_base_position('object')
        self.robot.set_action(action[:self.robot.action_space.shape[0]])
        self.sim.step()
        obs = self.get_obs()
        reward = self.compute_reward()
        done = False
        info = {
            'is_success': self.is_success()
        }
        return obs, reward, done, info

    def reset(self):
        sampled_position_xy = np.random.random_sample(size=(2,))
        sampled_position_xy *= self.object_range_xy
        sampled_position_xy -= self.object_range_xy/2.0
        object_position = np.concatenate([sampled_position_xy, [self.object_size/2.0]])
        target_position = np.concatenate([sampled_position_xy, [self.object_size/2.0 + 0.1]])
        self.sim.set_base_pose('object', object_position, np.array([0.0, 0.0, 0.0, 1.0]))
        self.sim.set_base_pose('target', target_position, np.array([0.0, 0.0, 0.0, 1.0]))
        self.previous_ee_position = None
        self.previous_object_position = None
        self.robot.reset()
        self.sim.step()
        obs = self.get_obs()
        return obs

    def get_obs(self):
        ee_position = self.robot.get_ee_position()
        robot_position = self.sim.get_base_position(self.robot.body_name)
        object_position = self.sim.get_base_position('object')
        object_velocity = self.sim.get_base_velocity('object')
        object_distance = object_position - ee_position
        return np.concatenate([self.robot.get_obs(), object_distance.astype(np.float32), object_velocity.astype(np.float32)])

    def compute_reward(self):
        object_position = self.sim.get_base_position('object')
        target_position = self.sim.get_base_position('target')
        ee_position = self.robot.get_ee_position()
        finger0_touch_object = len(p.getContactPoints(self.sim._bodies_idx[self.robot.body_name], self.sim._bodies_idx['object'], self.robot.finger_indices[0], physicsClientId=self.sim.physics_client._client)) > 0
        finger1_touch_object = len(p.getContactPoints(self.sim._bodies_idx[self.robot.body_name], self.sim._bodies_idx['object'], self.robot.finger_indices[1], physicsClientId=self.sim.physics_client._client)) > 0
        '''reward = 0.0
        if np.linalg.norm(object_position - target_position) < self.distance_threshold:
            reward += 10.0
        if self.previous_object_position is not None and np.linalg.norm(object_position - target_position) < np.linalg.norm(self.previous_object_position - target_position):
            reward += 2.0
        elif self.previous_object_position is not None and np.linalg.norm(object_position - target_position) > np.linalg.norm(self.previous_object_position - target_position):
            reward -= 2.0
        if np.linalg.norm(object_position - ee_position) < self.distance_threshold:
            reward += 1.0
        if finger0_touch_object and finger1_touch_object:
            reward += 1.0
        if self.previous_ee_position is not None and np.linalg.norm(object_position - ee_position) < np.linalg.norm(object_position - self.previous_ee_position):
            reward += 0.5
        elif self.previous_ee_position is not None and np.linalg.norm(object_position - ee_position) > np.linalg.norm(object_position - self.previous_ee_position):
            reward -= 0.5
        return reward'''
        reward = 0
        if np.linalg.norm(object_position - target_position) < self.distance_threshold:
            reward += 10
        if finger0_touch_object and finger1_touch_object:
            reward += 1
        return reward

    def create_scene(self):
        self.sim.create_plane(z_offset=-0.4)
        self.sim.create_table(length=0.7, width=0.7, height=0.4, x_offset=0.1)
        self.sim.create_box(
            body_name="object",
            half_extents=np.ones(3) * self.object_size / 2,
            mass=0.1,
            position=np.array([0.0, 0.0, self.object_size / 2]),
            rgba_color=np.array([0.1, 0.9, 0.1, 1.0]),
        )
        self.sim.create_box(
            body_name="target",
            half_extents=np.ones(3) * self.object_size / 2,
            mass=0.0,
            ghost=True,
            position=np.array([0.0, 0.0, self.object_size/2 + 0.1]),
            rgba_color=np.array([0.1, 0.9, 0.1, 0.3]),
        )

    def is_success(self):
        object_position = self.sim.get_base_position('object')
        target_position = self.sim.get_base_position('target')
        return 1.0*((np.linalg.norm(object_position - target_position) < self.distance_threshold))
