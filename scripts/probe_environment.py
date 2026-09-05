#!/usr/bin/env python3
"""Read ROS/Gazebo versions and actual binary dependencies without modifying the system."""

import argparse
import datetime
import json
import os
import platform
import shutil
import subprocess
from pathlib import Path


def run(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        return {"command": command, "returncode": result.returncode, "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"command": command, "error": str(exc)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    packages = [
        "ros-humble-desktop", "ros-humble-ros2cli", "ros-humble-moveit",
        "ros-humble-controller-manager", "ros-humble-gz-ros2-control",
        "ros-humble-ros-gzharmonic-bridge", "ros-humble-ros-gzharmonic-sim",
        "ros-humble-realsense2-description", "ros-humble-realsense2-camera",
        "gz-sim8-cli", "libignition-gazebo6",
    ]
    report = {
        "checked_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "platform": platform.platform(),
        "os_release": Path("/etc/os-release").read_text(),
        "environment": {k: os.environ.get(k) for k in ["ROS_VERSION", "ROS_DISTRO", "ROS_PYTHON_VERSION", "RMW_IMPLEMENTATION", "AMENT_PREFIX_PATH", "DISPLAY", "WAYLAND_DISPLAY"]},
        "executables": {x: shutil.which(x) for x in ["ros2", "gz", "ign", "gazebo", "colcon", "xacro", "nvidia-smi"]},
        "checks": [],
    }
    for command in [
        ["gz", "sim", "--versions"], ["ign", "gazebo", "--versions"],
        ["dpkg-query", "-W", "-f=${Package} ${Version}\n", *packages],
        ["nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"],
    ]:
        report["checks"].append(run(command))
    for package, filename in [
        ("gz_ros2_control", "lib/libgz_ros2_control-system.so"),
        ("ros_gz_bridge", "lib/ros_gz_bridge/parameter_bridge"),
    ]:
        prefix = run(["ros2", "pkg", "prefix", package])
        report["checks"].append(prefix)
        if prefix.get("returncode") == 0:
            linkage = run(["ldd", str(Path(prefix["stdout"]) / filename)])
            linkage["stdout"] = "\n".join(line for line in linkage.get("stdout", "").splitlines() if any(key in line for key in ["libgz-", "libignition-", "not found"]))
            report["checks"].append(linkage)
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
        print(f"Environment evidence written to {args.output}")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
