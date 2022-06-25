from panda_gym.envs.tasks.pick_and_place import PickAndPlace
from panda_gym.utils import distance
import numpy as np
from typing import Dict, Any, Union
from manipulation_drl.doris_robot import DoRISRobot

class DoRISPickAndPlaceTask(PickAndPlace):
    def __init__(self, sim, reward_type: str = "sparse", distance_threshold: float = 0.05, goal_xy_range: float = 0.3, goal_z_range: float = 0.2, obj_xy_range: float = 0.3, get_ee_position = None) -> None:
        super().__init__(sim, reward_type, distance_threshold, goal_xy_range, goal_z_range, obj_xy_range)
        self.get_ee_position = get_ee_position
    
    def _create_scene(self) -> None:
        """Create the scene."""
        self.sim.create_plane(z_offset=-0.4)
        self.sim.create_table(length=0.7, width=0.7, height=0.4, x_offset=0.1)
        self.sim.create_box(
            body_name="object",
            half_extents=np.ones(3) * self.object_size / 2,
            mass=1.0,
            position=np.array([0.0, 0.0, self.object_size / 2]),
            rgba_color=np.array([0.1, 0.9, 0.1, 1.0]),
        )
        self.sim.create_box(
            body_name="target",
            half_extents=np.ones(3) * self.object_size / 2,
            mass=0.0,
            ghost=True,
            position=np.array([0.0, 0.0, 0.05]),
            rgba_color=np.array([0.1, 0.9, 0.1, 0.3]),
        )

    @classmethod
    def reward_function(self, achieved_goal, desired_goal, distance_threshold):
        d = distance(achieved_goal, desired_goal)
        return -np.array(d > distance_threshold, dtype=np.float64)

    def compute_reward(self, achieved_goal, desired_goal, info: Dict[str, Any]) -> Union[np.ndarray, float]:
        d = distance(achieved_goal, desired_goal)
        if self.reward_type == "sparse":
            return -np.array(d > self.distance_threshold, dtype=np.float64)
        elif self.reward_type == "shaped":
            ee_position = self.get_ee_position()
            object_position = self.sim.get_base_position('object')
            reward = -d + 0.3 -distance(ee_position, object_position)
            return reward
        else:
            return -d

    def change_level(self, level):
        self.obj_range_low -= 0.05 * level
        self.obj_range_high += 0.05 * level
        self.goal_range_low -= 0.05 * level
        self.goal_range_high += 0.05 * level
