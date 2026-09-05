#!/usr/bin/env bash
# Source this file in project shells. Scope ROS and Gazebo discovery to this demo.
MTC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
if [ -f "$MTC_ROOT/vendor_ws/install/setup.bash" ]; then source "$MTC_ROOT/vendor_ws/install/setup.bash"; fi
if [ -f "$MTC_ROOT/install/setup.bash" ]; then source "$MTC_ROOT/install/setup.bash"; fi
export ROS_DOMAIN_ID="${MTC_ROS_DOMAIN_ID:-73}"
export CYCLONEDDS_URI="file://$MTC_ROOT/config/cyclonedds.xml"
# CycloneDDS already binds lo in the project XML; ROS_LOCALHOST_ONLY would add it twice.
export ROS_LOCALHOST_ONLY=0
export GZ_PARTITION="${MTC_GZ_PARTITION:-mtc_s3_demo}"
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$MTC_ROOT/install/mtc_simulation/lib:$MTC_ROOT/vendor_ws/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$MTC_ROOT/src:$MTC_ROOT/src/mtc_simulation/models:/opt/ros/humble/share:${GZ_SIM_RESOURCE_PATH:-}"
