#!/usr/bin/env python

import math
import sys

import rospy
from moveit_apps.srv import PickPlaceService, PickPlaceServiceRequest

services = {
    'pick': '/butia_moveit/tasks/pick',
    'place': '/butia_moveit/tasks/place',
    'both': '/butia_moveit/tasks/pick_place',
}

if __name__ == '__main__':
    rospy.init_node('demo_pickplace')
    if len(sys.argv) < 2:
        print("Usage: demo_pickplace.py pick/place/both")
    else:
        pickplace = rospy.ServiceProxy(services[sys.argv[1]], PickPlaceService)
        request = PickPlaceServiceRequest()
        request.object_name = 'box'
        request.object_pose.header.frame_id = 'world'
        request.object_pose.pose.pose.position.x = 0.5
        request.object_pose.pose.pose.position.y = -0.25
        request.object_pose.pose.pose.position.z = 0.8 + (0.25/2) + (0.1/2)
        request.object_pose.pose.pose.orientation.x = 0
        request.object_pose.pose.pose.orientation.y = 0
        request.object_pose.pose.pose.orientation.z = 0
        request.object_pose.pose.pose.orientation.w = 1
        request.object_pose.pose.covariance[0] = 0.04
        request.object_pose.pose.covariance[7] = 0.04
        request.object_pose.pose.covariance[16] = 0.25
        request.support_surface_name = 'table'
        request.surface_pose.header.frame_id = 'world'
        request.surface_pose.pose.pose.position.x = 0.5
        request.surface_pose.pose.pose.position.y = -0.25
        request.surface_pose.pose.pose.position.z = 0.8
        request.surface_pose.pose.pose.orientation.x = 0
        request.surface_pose.pose.pose.orientation.y = 0
        request.surface_pose.pose.pose.orientation.z = 0
        request.surface_pose.pose.pose.orientation.w = 1
        request.surface_pose.pose.covariance[0] = 0.4
        request.surface_pose.pose.covariance[7] = 0.5
        request.surface_pose.pose.covariance[16] = 0.1
        request.place_pose.header.frame_id = 'world'
        request.place_pose.pose.position.x = 0.6
        request.place_pose.pose.position.y = -0.15
        request.place_pose.pose.position.z = 0.928
        request.place_pose.pose.orientation.x = 0
        request.place_pose.pose.orientation.y = 0
        request.place_pose.pose.orientation.z = 0
        request.place_pose.pose.orientation.w = 1
        pickplace(request)