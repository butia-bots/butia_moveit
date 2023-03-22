#!/bin/sh

[ $(which moveit) ] || sudo apt install -y ros-noetic-moveit
[ $(which moveit_visual_tools) ] || sudo apt-get install -y ros-noetic-moveit-visual-tools
[ $(which dynamixel_sdk) ] || sudo apt-get install -y ros-noetic-dynamixel-sdk