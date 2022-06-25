from manipulation_drl.pick_and_place_task import DoRISPickAndPlaceTask
from panda_gym.envs.core import RobotTaskEnv
from panda_gym.pybullet import PyBullet
from manipulation_drl.doris_robot import DoRISRobot
import numpy as np

class DoRISPickAndPlaceEnv(RobotTaskEnv):
    def __init__(self, render: bool = False, reward_type: str = "sparse", **kwargs):
        sim = PyBullet(render=render)
        robot = DoRISRobot(sim)
        task = DoRISPickAndPlaceTask(sim, reward_type=reward_type, get_ee_position=robot.get_ee_position)
        super().__init__(robot, task)
        self.observation_space['observation'].low = -50*np.ones(shape=self.observation_space['observation'].shape)
        self.observation_space['observation'].high = 50*np.ones(shape=self.observation_space['observation'].shape)
        self.observation_space['desired_goal'].low = -50*np.ones(shape=self.observation_space['desired_goal'].shape)
        self.observation_space['desired_goal'].high = 50*np.ones(shape=self.observation_space['desired_goal'].shape)
        self.observation_space['achieved_goal'].low = -50*np.ones(shape=self.observation_space['achieved_goal'].shape)
        self.observation_space['achieved_goal'].high = 50*np.ones(shape=self.observation_space['achieved_goal'].shape)
    
    def change_level(self, level):
        self.task.change_level(level)
