#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs
from moveit_apps.srv import PickPlaceService, PickPlaceServiceRequest, PickPlaceServiceResponse
import sys
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
import tf

def wait_for_collision_update(box_name, box_is_attached=False, box_is_known=True):
    start = rospy.get_time()
    seconds = rospy.get_time()
    timeout = 5.0
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


def handle_pick(req):
    obj_pose = PoseStamped()
    obj_pose.pose = req.object_pose.pose.pose
    obj_pose.header.frame_id = robot.get_planning_frame()
    scene.add_box(
        req.object_name,
        obj_pose,
        size=(
            req.object_pose.pose.covariance[0],
            req.object_pose.pose.covariance[7],
            req.object_pose.pose.covariance[16]
        )
    )
    rospy.loginfo(req.object_pose.pose.pose)
    rospy.loginfo(wait_for_collision_update(req.object_name))
    surface_pose = PoseStamped()
    surface_pose.pose = req.surface_pose.pose.pose
    surface_pose.header.frame_id = robot.get_planning_frame()
    scene.add_box(
        req.support_surface_name,
        surface_pose,
        size=(
            req.surface_pose.pose.covariance[0],
            req.surface_pose.pose.covariance[7],
            req.surface_pose.pose.covariance[16]
        )
    )
    rospy.loginfo(wait_for_collision_update(req.support_surface_name))
    gripper_move_group.set_named_target('open')
    gripper_move_group.go()
    ee_pose = PoseStamped()
    ee_pose.header.frame_id = obj_pose.header.frame_id
    ee_pose.pose.position = obj_pose.pose.position
    ee_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(-1.570, 0, 0))
    ee_pose.pose.position.y += 0.15
    move_group.set_pose_target(ee_pose, end_effector_link='gripper_link')
    move_group.go()
    ee_pose.pose.position.y -= 0.1
    move_group.set_pose_target(ee_pose, end_effector_link='gripper_link')
    move_group.go()
    grasping_group = 'gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box('gripper_link', req.object_name, touch_links=touch_links)
    gripper_move_group.set_named_target('close')
    gripper_move_group.go()
    move_group.set_named_target('home')
    move_group.go()
    res = PickPlaceServiceResponse()
    return res

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_place_service')
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander('arm')
    gripper_move_group = moveit_commander.MoveGroupCommander('gripper')
    scene = moveit_commander.PlanningSceneInterface()
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    pick_service = rospy.Service('/butia_moveit/tasks/pick', PickPlaceService, handle_pick)
    rospy.spin()
