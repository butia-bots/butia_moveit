import math
import numpy as np
from gym import spaces
import rospkg
from panda_gym.envs.core import PyBulletRobot
from panda_gym.pybullet import PyBullet
import pybullet as p

rospack = rospkg.RosPack()

class DoRISRobot(PyBulletRobot):
    """DoRIS robot"""

    def __init__(self, sim):
        action_dim = 4
        action_space = spaces.Box(-1.0, 1.0, shape=(action_dim,), dtype=np.float32)
        #self.lock_orientation = np.array([0, 1, 0, 0])
        sim.physics_client.setAdditionalSearchPath(rospack.get_path('manipulation_drl')+'/assets')
        self.ee_link = 16
        super().__init__(
            sim,
            body_name="doris",
            file_name=rospack.get_path('manipulation_drl')+'/assets/doris_description_headless.urdf',
            #file_name='./doris_description_nonexistent.urdf',
            base_position=np.array([-0.6, 0.0, -0.35]),
            action_space=action_space,
            #joint_indices=np.array([6, 7, 8, 9, 10, 11, 12, 14, 15]),
            joint_indices=np.array([6, 7, 8, 9, 10, 11, 12, 17, 18]),
            joint_forces=np.array([1000.0,]*9),
        )
        self.finger_indices = np.array([17, 18])
        self.sim.set_lateral_friction(self.body_name, self.finger_indices[0], lateral_friction=1.0)
        self.sim.set_lateral_friction(self.body_name, self.finger_indices[1], lateral_friction=1.0)
        self.sim.set_spinning_friction(self.body_name, self.finger_indices[0], spinning_friction=0.001)
        self.sim.set_spinning_friction(self.body_name, self.finger_indices[1], spinning_friction=0.001)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(7,), dtype=np.float32)
        #print(p.getJointInfo(0, 13))
        for i in range(20):
            print(p.getJointInfo(0, i))
        '''self.object = PyBullet().create_box('object', (0.05, 0.05, 0.05), 0.1, (0.5, 0.0, 0.75), (0, 0, 0, 255))
        self.target = PyBullet().create_box('object', (0.05, 0.05, 0.05), 0.1, (0.5, 0.0, 1.0), (255, 0, 0, 255), ghost=True)
        self.table = PyBullet().create_box('table', (0.2, 0.6, 0.35), 0.1, (0.5, 0.0, 0.35), (255, 255, 255, 255))'''

    def set_action(self, action):
        action = action.copy()
        action = np.clip(action, self.action_space.low, self.action_space.high)
        ee_action = action[:3]
        gripper_action = action[3]
        ee_action *= 0.05
        ee_position_action = ee_action[:3]
        #ee_orientation_action = ee_action[3:]
        gripper_position = self.get_link_position(link=self.ee_link)
        gripper_position += ee_position_action
        gripper_orientation = p.getQuaternionFromEuler((0.0, math.pi/2, 0.0))
        #gripper_orientation += ee_orientation_action
        #gripper_orientation = self.lock_gripper_orientation
        arm_joint_angles = np.array(self.sim.inverse_kinematics(self.body_name, link=self.ee_link, position=gripper_position, orientation=gripper_orientation))
        arm_joint_angles = arm_joint_angles[2:9]
        arm_joint_angles[0] = 0.0
        #arm_joint_angles = arm_joint_angles[self.joint_indices][:-1]
        #arm_joint_angles[0] = np.clip(arm_joint_angles[0], 0.0, 0.8)
        #arm_joint_angles[0] = 0.5
        finger1 = self.sim.get_joint_angle(self.body_name, self.joint_indices[-2])
        finger2 = self.sim.get_joint_angle(self.body_name, self.joint_indices[-1])
        opening = (abs(finger1)+abs(finger2))
        opening += gripper_action
        gripper_joint_angles = [-opening/2, opening/2]
        #gripper_joint_angles = [opening + gripper_action, -(opening + gripper_action)]
        #gripper_joint_angles = np.clip(gripper_joint_angles, -0.08, 0.08)
        #self.control_joints(np.concatenate([arm_joint_angles, gripper_joint_angles]))
        self.control_joints(np.concatenate([arm_joint_angles, gripper_joint_angles]))
        #action *= 0.05
        #current_angles = np.array([self.get_joint_angle(idx) for idx in self.joint_indices])
        #target_angles = current_angles + action
        #self.control_joints(target_angles=target_angles)

    def get_obs(self):
        base_position = self.sim.get_base_position(self.body_name)
        gripper_position = self.get_link_position(link=self.ee_link)
        gripper_velocity = self.get_link_velocity(link=self.ee_link)
        finger1 = self.sim.get_joint_angle(self.body_name, self.joint_indices[-2])
        finger2 = self.sim.get_joint_angle(self.body_name, self.joint_indices[-1])
        opening = (abs(finger1)+abs(finger2))
        obs = np.concatenate((gripper_position, gripper_velocity, [opening,]))
        #obs = np.array([self.get_joint_angle(idx) for idx in self.joint_indices])
        return obs

    def reset(self):
        '''arm_joint_angles = np.array(self.sim.inverse_kinematics('doris', link=self.ee_link, position=[-0.2, 0.0, 0.2], orientation=self.lock_orientation))
        arm_joint_angles = arm_joint_angles[self.joint_indices][:-2]
        arm_joint_angles[0] = np.clip(arm_joint_angles[0], 0.0, 0.8)
        gripper_joint_angles = [0.6, 0.6]
        self.set_joint_angles(np.concatenate([arm_joint_angles, gripper_joint_angles]))'''
        joint_angles = [0.0,]*len(self.joint_indices)
        joint_angles[-2:-1] = [-0.05, -0.05]
        self.set_joint_angles(joint_angles)

    def get_ee_position(self):
        gripper_position = self.get_link_position(link=self.ee_link)
        return gripper_position
