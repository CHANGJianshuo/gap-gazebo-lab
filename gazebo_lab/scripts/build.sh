#!/usr/bin/env bash
set -euo pipefail
GAP_BUILD_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Requires installed ROS Humble, MoveIt, Gazebo Harmonic development libraries.
set +u
source /opt/ros/humble/setup.bash
set -u
export GZ_VERSION=harmonic PYTHONNOUSERSITE=1
/usr/bin/python3 "$GAP_BUILD_ROOT/scripts/prepare.py"
/usr/bin/cmake -S "$GAP_BUILD_ROOT/deps/gz_ros2_control/gz_ros2_control" -B "$GAP_BUILD_ROOT/deps/control_build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$GAP_BUILD_ROOT/deps/control_install"
/usr/bin/cmake --build "$GAP_BUILD_ROOT/deps/control_build" -j 4
/usr/bin/cmake --install "$GAP_BUILD_ROOT/deps/control_build"
/usr/bin/cmake -S "$GAP_BUILD_ROOT" -B "$GAP_BUILD_ROOT/build" -DCMAKE_BUILD_TYPE=Release
/usr/bin/cmake --build "$GAP_BUILD_ROOT/build" -j 4
