import gym
import gym.spaces
from typing import Tuple, Dict, Any

from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.transform import Rotation
from copy import deepcopy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import rospy

class DoRISManipulationEnv(gym.Env):
    def __init__(self) -> None:
        super(DoRISManipulationEnv, self).__init__()
        '''self.manipulator = InterbotixManipulatorXS("doris_arm", init_node=False)
        self.manipulator.arm.moving_time = 0.05'''
        self.debug_pub = rospy.Publisher('/butia_moveit/drl/goal', PoseStamped)
        self.arm_joint_names = [
            'waist',
            'shoulder',
            'elbow',
            'wrist_angle',
            'wrist_rotate'
        ]
        self.arm_joint_angles = np.array([0.0,]*len(self.arm_joint_names))
        self.arm_joint_publishers = [rospy.Publisher("/doris_arm/{}_controller/command".format(joint_name), Float64) for joint_name in self.arm_joint_names]
        self.joint_state_subscriber = rospy.Subscriber("/doris_arm/joint_states", JointState, self.handle_joint_state)
        self.initial_guesses = [[0.0] * len(self.arm_joint_names) for i in range(3)]
        self.initial_guesses[1][0] = np.deg2rad(-120)
        self.initial_guesses[2][0] = np.deg2rad(120)
        self.Slist = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, -0.12705, 0.0, 0.0],
                        [0.0, 1.0, 0.0, -0.42705, 0.0, 0.05955],
                        [0.0, 1.0, 0.0, -0.42705, 0.0, 0.35955],
                        [1.0, 0.0, 0.0, 0.0, 0.42705, 0.0]]).T

        self.M = np.array([[1.0, 0.0, 0.0, 0.536494],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.42705],
                    [0.0, 0.0, 0.0, 1.0]])

    def handle_joint_state(self, msg):
        for i in range(len(self.arm_joint_names)):
            i_joint_msg = msg.name.index(self.arm_joint_names[i])
            self.arm_joint_angles[i] = msg.position[i_joint_msg]
    
    def _get_target(self) -> np.ndarray:
        pass
    
    def _get_obs(self):
        pass

    def _compute_reward(self):
        pass
    
    def reset(self) -> Any:
        pass

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        pass

