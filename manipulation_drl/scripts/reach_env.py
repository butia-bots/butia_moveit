import gym
import gym.spaces
from typing import Tuple, Dict, Any

import moveit_commander
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.transform import Rotation
from copy import deepcopy

import rospy

class DoRISReachEnv(gym.Env):
    def __init__(self) -> None:
        super(DoRISReachEnv, self).__init__()
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(14,))
        self.action_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(7,))
        self.arm_move_group = moveit_commander.MoveGroupCommander('arm', wait_for_servers=1000)
        self.gripper_move_group = moveit_commander.MoveGroupCommander('gripper', wait_for_servers=1000)
        self.rail_move_group = moveit_commander.MoveGroupCommander('rail', wait_for_servers=1000)
        #self.workspace_limits = [0.0, -0.5, 0.5, 0.5, 0.5, 1.0]
        #self.workspace_range = [
        #    self.workspace_limits[3] - self.workspace_limits[0],
        #    self.workspace_limits[4] - self.workspace_limits[1],
        #    self.workspace_limits[5] - self.workspace_limits[2],
        #]
        #self.arm_move_group.set_workspace(self.workspace_limits)
        self.arm_move_group.set_planning_time(0.1)
        #self.workspace_limits = np.array(self.workspace_limits)
        self.debug_pub = rospy.Publisher('/butia_moveit/drl/goal', PoseStamped)

    def _pose2vec(self, pose: Pose) -> np.ndarray:
        """
        Converts pose message into ndarray ordered as: (px, py, pz, ox, oy, oz, ow)
        """
        rot = Rotation.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        '''pose_vec = np.array([
            (pose.position.x - self.workspace_limits[0])/self.workspace_range[0],
            (pose.position.y - self.workspace_limits[1])/self.workspace_range[1],
            (pose.position.z - self.workspace_limits[2])/self.workspace_range[2],
            *rot.as_quat()
        ])'''
        pose_vec = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
            *rot.as_quat()
        ])
        #pose_vec[:3] = (pose_vec[:3] - 0.5) * 2
        return pose_vec

    def _vec2pose(self, vec: np.ndarray) -> Pose:
        """
        Converts ndarray ordered as (px, py, pz, ox, oy, oz, ow) into pose message
        """
        rot = Rotation.from_quat(vec[3:])
        quat = rot.as_quat()
        pose = Pose()
        #vec = (vec + 1)/2
        pose.position.x = float(vec[0])
        pose.position.y = float(vec[1])
        pose.position.z = float(vec[2])
        '''pose.position.x = float(vec[0]/self.workspace_range[0] + self.workspace_limits[0])
        pose.position.y = float(vec[1]/self.workspace_range[1] + self.workspace_limits[1])
        pose.position.z = float(vec[2]/self.workspace_range[2] + self.workspace_limits[2])'''
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose
    
    def _get_target(self) -> np.ndarray:
        target = np.random.random_sample(size=(7,))
        target[2] /= 2.0
        target[2] += 0.5
        target[0] /= 2.0
        target[0] += 0.5
        target[1] -= 0.5
        return target 
    
    def _get_obs(self):
        current_ee_pose = self.arm_move_group.get_current_pose(end_effector_link='gripper_link')
        current_ee_pose = self._pose2vec(current_ee_pose.pose)
        self.current_pose = current_ee_pose
        obs = np.concatenate([current_ee_pose, self.target_pose])
        ps = PoseStamped()
        ps.header.frame_id = "base_footprint"
        ps.pose = self._vec2pose(self.target_pose)
        self.debug_pub.publish(ps)
        return obs

    def _compute_reward(self):
        return 10*(1 - np.linalg.norm(self.current_pose - self.target_pose))
    
    def reset(self) -> Any:
        self.rail_move_group.set_named_target('home')
        self.rail_move_group.go()
        self.gripper_move_group.set_named_target('home')
        self.gripper_move_group.go()
        self.arm_move_group.set_named_target('home')
        self.arm_move_group.go()
        target_ee_pose = self._get_target()
        self.target_pose = target_ee_pose
        return self._get_obs()

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        waypoints = []
        ee_pose = self.arm_move_group.get_current_pose().pose
        waypoints.append(deepcopy(ee_pose))
        ee_pose.position.x += float(0.05*action[0])
        ee_pose.position.y += float(0.05*action[1])
        ee_pose.position.z += float(0.05*action[2])
        ee_pose.orientation.x += float(0.05*action[3])
        ee_pose.orientation.y += float(0.05*action[4])
        ee_pose.orientation.z += float(0.05*action[5])
        ee_pose.orientation.w += float(0.05*action[6])
        waypoints.append(ee_pose)
        path, fraction = self.arm_move_group.compute_cartesian_path(waypoints, eef_step=0.001, jump_threshold=0.0, avoid_collisions=False)
        #self.arm_move_group.set_pose_target(ee_pose.pose, end_effector_link='gripper_link')
        self.arm_move_group.execute(path)
        obs = self._get_obs()
        reward = self._compute_reward()
        info = {}
        done = False
        return obs, reward, done, info

