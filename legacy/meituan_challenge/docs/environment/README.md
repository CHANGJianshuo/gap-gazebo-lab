# 环境检查 · 2026-09-05

证据：[observed.json](observed.json)，可由 `scripts/probe_environment.py` 重现。

| 项目 | 本机结果 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS，WSL2，x86_64 |
| ROS | **ROS 2 Humble**，`/opt/ros/humble` |
| ROS 环境变量 | `ROS_VERSION=2`，`ROS_DISTRO=humble` |
| DDS | `rmw_cyclonedds_cpp` |
| Gazebo Harmonic | `gz sim --versions` → **8.12.0** |
| Gazebo Fortress | `ign gazebo --versions` → **6.17.1** |
| Gazebo Classic | 未发现 `gazebo` 可执行文件 |
| MoveIt 2 | `ros-humble-moveit` **2.5.9** |
| ros2_control | controller manager **2.54.0** |
| gz_ros2_control | apt 包 **0.7.19**，实际链接 Fortress |
| ros_gz bridge | `ros-humble-ros-gzharmonic-bridge` **0.244.12**，实际链接 Harmonic |
| RealSense | camera / description **4.57.7** |
| GPU | RTX 5060 Laptop，8151 MiB，驱动 592.27 |
| 显示环境 | `DISPLAY=:0`，`WAYLAND_DISPLAY=wayland-0` |

`ros2` 没有统一代表发行版的 `--version` 用法；这里通过环境变量、安装路径和软件包版本共同确认。

二进制依赖检查发现实质性不匹配：

```text
/opt/ros/humble/lib/libgz_ros2_control-system.so
  → libignition-gazebo6.so.6
  → libignition-transport11.so.11

/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge
  → libgz-transport13.so.13
  → libgz-msgs10.so.10
```

因此不能仅凭两个包都安装了就判断整个控制链已经能在 Harmonic 运行。

建议沿用当前 ROS 2 Humble + Gazebo Harmonic，按 [gz_ros2_control 官方 Humble 文档](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)在项目工作空间编译 Harmonic 版本的控制插件，设置 `GZ_VERSION=harmonic`，固定源码版本并核对最终链接库。已有系统软件包保持现状，项目启动时显式加载自己的 overlay。

[Gazebo 官方兼容说明](https://gazebosim.org/docs/harmonic/ros_installation/)将 Humble + Fortress 列为默认配对，Humble + Harmonic 为需要额外适配的可用组合。因此本建议来自本机已安装的 Harmonic bridge 和开发库，不代表它是 Humble 的默认组合。

已执行一次独立分区的服务端冒烟检查：

```bash
GZ_PARTITION=meituan_environment_probe gz sim -s -r --iterations 10 /usr/share/gz/gz-sim8/worlds/empty.sdf
```

退出码为 0。它只证明空场景服务端能运行；尚未验证 GUI、RGB-D 渲染、ROS 图像桥接、S3 加载或关节控制。GPU 查询成功也不等于 Gazebo 已使用硬件加速渲染。


## 2026-09-05 已实现的运行 overlay

系统安装状况仍保留上述取证记录。实际演示通过 `scripts/env.sh` 加载 `vendor_ws/install`：gz_ros2_control 固定提交 c88a5fd9170af120c263c1201f0744a40f93d673，重新链接 Harmonic 的 libgz-sim8；JTC 固定 2.53.1 提交 159e6298b2a308b1cb3596680e98b86eb80dedc3，增加 effort 前馈。补丁在 scripts，系统 /opt/ros 未改动。

项目使用 ROS_DOMAIN_ID=73、独立 Gazebo partition 和环回 CycloneDDS 配置。头less Ogre2 在本机可渲染，实际仿真约以 0.09 倍墙钟速度推进；录像仍保持 1 倍仿真时间，不冒充墙钟实时性能。
