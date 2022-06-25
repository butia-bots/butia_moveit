from panda_gym.envs.core import RobotTaskEnv
from reach_task import DoRISReach
from panda_gym.pybullet import PyBullet
from doris_robot import DoRISRobot

class DoRISReachEnv(RobotTaskEnv):
    def __init__(self, render: bool = False, reward_type: str = "sparse"):
        sim = PyBullet(render=render)
        robot = DoRISRobot(sim)
        task = DoRISReach(sim, robot.get_ee_position, reward_type=reward_type)
        super().__init__(robot, task)
