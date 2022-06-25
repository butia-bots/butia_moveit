import gym
import gym.spaces
from typing import Tuple, Dict, Any

import moveit_commander
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.transform import Rotation
from copy import deepcopy
from manipulation_env import DoRISManipulationEnv
from modern_robotics.core import IKinSpace, FKinSpace
from ray.rllib.env.external_env import ExternalEnv

import rospy

class DoRISReachEnv(DoRISManipulationEnv):
    def __init__(self) -> None:
        super(DoRISReachEnv, self).__init__()
        self.observation_space = gym.spaces.Box(low=-10.0, high=10.0, shape=(12+5,))
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(5,))
        self.use_ik = False
    
    def _get_target(self) -> np.ndarray:
        target = np.random.random_sample(size=(6,))
        #target[2] /= 2.0
        #target[2] += 0.5
        #target[0] /= 2.0
        target[0] -= 0.5
        target[1] -= 0.5
        target[3:] *= 4*np.math.pi
        target[3:] -= 2*np.math.pi
        return target * 0.6
    
    def _get_obs(self):
        pose_matrix = FKinSpace(self.M, self.Slist, self.arm_joint_angles)
        rot_matrix = pose_matrix[:3,:3]
        rot = Rotation.from_matrix(rot_matrix)
        rpy = rot.as_euler('xyz')
        position = pose_matrix[:,3].flatten()[:-1]
        current_ee_pose = np.concatenate([position, rpy])
        self.current_pose = current_ee_pose
        obs = np.concatenate([current_ee_pose, self.target_pose, self.arm_joint_angles])
        ps = PoseStamped()
        ps.header.frame_id = "doris_arm/base_link"
        ps.pose.position.x = self.target_pose[0]
        ps.pose.position.y = self.target_pose[1]
        ps.pose.position.z = self.target_pose[2]
        #rot = Rotation.from_euler('xyz', np.zeros(3))
        rot = Rotation.from_euler('xyz', self.target_pose[3:])
        quat = rot.as_quat()
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        self.debug_pub.publish(ps)
        return obs

    def _goal_distance(self, a, b):
        return np.linalg.norm(a - b)

    def _compute_reward(self):
        #return 10*(1 - np.linalg.norm(self.current_pose - self.target_pose))
        #previous_distance = self._goal_distance(self.target_pose, self.previous_pose)
        current_distance = self._goal_distance(self.target_pose[:-1], self.current_pose[:-1])
        reward = 10 if current_distance < 0.05 else 0
        return reward
    
    def reset(self) -> Any:
        #self.manipulator.gripper.open()
        for joint_pub in self.arm_joint_publishers:
            joint_pub.publish(0.0)
        rate = rospy.Rate(0.5)
        rate.sleep()
        target_ee_pose = self._get_target()
        self.target_pose = target_ee_pose
        return self._get_obs()

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        #action[4] = 0
        pose_matrix = FKinSpace(self.M, self.Slist, self.arm_joint_angles)
        rot_matrix = pose_matrix[:3,:3]
        rot = Rotation.from_matrix(rot_matrix)
        rpy = rot.as_euler('xyz')
        position = pose_matrix[:,3].flatten()[:-1]
        ee_pose = np.concatenate([position, rpy])
        self.previous_pose = ee_pose
        ee_pose[0] += float(0.05*action[0])
        #ee_pose[1] += float(0.05*action[1])
        ee_pose[2] += float(0.05*action[2])
        #ee_pose[3] += float(0.05*action[3])
        #ee_pose[4] += float(0.05*action[4])
        #ee_pose[5] += float(0.05*action[5])
        rot = Rotation.from_euler('xyz', ee_pose[3:])
        pose_matrix[:3,:3] = rot.as_matrix()
        pose_matrix[:3,3] = ee_pose[:3]
        '''for guess in self.initial_guesses:
            theta_list, success = IKinSpace(self.Slist, self.M, pose_matrix, guess, 0.001, 0.001)
            theta_list[0] += 0.05*action[1]
            if success:
                break
        if success:
            for i in range(len(theta_list)):
                self.arm_joint_publishers[i].publish(theta_list[i])'''
        for i in range(len(self.arm_joint_angles)):
            self.arm_joint_publishers[i].publish(self.arm_joint_angles[i] + 0.05*action[i])
        rate = rospy.Rate(50)
        rate.sleep()
        obs = self._get_obs()
        reward = self._compute_reward()
        info = {'is_success': self._goal_distance(self.target_pose, self.current_pose) < 0.05}
        done = False
        return obs, reward, done, info

class DoRISEnvExternal(ExternalEnv):
    def __init__(self, env, max_concurrent: int = 100):
        self.doris_env = env
        super().__init__(self.doris_env.action_space, self.doris_env.observation_space, max_concurrent)
