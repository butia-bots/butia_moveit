#!/usr/bin/env bash

rm -rf `rospack find manipulation_drl`/assets
mkdir `rospack find manipulation_drl`/assets
cp `rospack find doris_description`/urdf/doris_description_viper_arm.urdf `rospack find manipulation_drl`/assets/doris_description_viper_arm.urdf
#cp -r `rospack find doris_description`/meshes `rospack find manipulation_drl`/assets/doris_description/meshes
#cp -r `rospack find open_manipulator_description`/meshes `rospack find manipulation_drl`/assets/open_manipulator_description/meshes
mkdir -p `rospack find manipulation_drl`/assets/doris_dorso_description/meshes
cp -r `rospack find doris_dorso_description`/meshes `rospack find manipulation_drl`/assets/doris_dorso_description
mkdir -p `rospack find manipulation_drl`/assets/doris_head_description/meshes
cp -r `rospack find doris_head_description`/meshes `rospack find manipulation_drl`/assets/doris_head_description
mkdir -p `rospack find manipulation_drl`/assets/doris_arm_description/meshes
cp -r `rospack find doris_arm_description`/meshes `rospack find manipulation_drl`/assets/doris_arm_description
mkdir -p `rospack find manipulation_drl`/assets/patrolbot_description/meshes
cp -r `rospack find patrolbot_description`/meshes `rospack find manipulation_drl`/assets/patrolbot_description
mkdir -p `rospack find manipulation_drl`/assets/hector_sensors_description/meshes
cp -r `rospack find hector_sensors_description`/meshes `rospack find manipulation_drl`/assets/hector_sensors_description