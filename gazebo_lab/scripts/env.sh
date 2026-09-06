#!/usr/bin/env bash
GAP_LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=gap_lab GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GAP_LAB_ROOT/build:/opt/ros/humble/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$GAP_LAB_ROOT/deps/moveit_resources:/opt/ros/humble/share:${GZ_SIM_RESOURCE_PATH:-}"
unset CYCLONEDDS_URI
if [ -f "$GAP_LAB_ROOT/deps/control_install/local_setup.bash" ]; then source "$GAP_LAB_ROOT/deps/control_install/local_setup.bash"; fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GAP_LAB_ROOT/deps/control_install/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export AMENT_PREFIX_PATH="$GAP_LAB_ROOT/deps/control_install:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="$GAP_LAB_ROOT/deps/control_install/lib:$LD_LIBRARY_PATH"
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$GAP_LAB_ROOT/config/cyclonedds.xml"
