#!/usr/bin/env bash
set -e
MTC_BUILD_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$MTC_BUILD_ROOT"
export PYTHONNOUSERSITE=1
export PATH="/usr/bin:/bin:$PATH"
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
python3 scripts/fetch_control_sources.py
python3 scripts/patch_control_limits.py
python3 scripts/patch_trajectory_effort.py
colcon --log-base vendor_ws/log build --base-paths vendor_ws/src --build-base vendor_ws/build --install-base vendor_ws/install --merge-install --packages-select gz_ros2_control joint_trajectory_controller --allow-overriding gz_ros2_control joint_trajectory_controller --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source vendor_ws/install/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source scripts/env.sh
mkdir -p data/validation
ros2 run mtc_motion_planning trajectory_self_test > data/validation/trajectory_math.txt
ros2 run mtc_motion_planning controller_interpolation_test > data/validation/controller_interpolation.json
ros2 run mtc_motion_planning short_moves_test > data/validation/short_moves.txt
if python3 -c 'import pybullet' >/dev/null 2>&1; then
  python3 scripts/check_dynamics.py
else
  echo 'Optional Bullet cross-check skipped: install pybullet==3.2.7 to run scripts/check_dynamics.py'
fi
