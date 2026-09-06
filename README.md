# AUBO S3 运动规划核心

ROS 2 Humble + MoveIt 2 + Gazebo Harmonic。以仿真真值为输入，规划并执行带半圆锁扣的电池抓放。只维护 `main` 分支。

完整历史、GaP 研究、原始 CAD/PDF、教学报告和历史录像已归档至 [gap-gazebo-lab](https://github.com/CHANGJianshuo/gap-gazebo-lab/tree/main/legacy)。本地完整版本位于 `/home/chang/gap`。

## 保留内容

| 目录 | 功能 |
| --- | --- |
| `src/mtc_motion_planning` | IK、Ruckig/B 样条候选、RRT-Connect 后备、碰撞与动力学约束、轨迹 Action、核心测试 |
| `src/mtc_interfaces` | 规划 Action 和锁扣服务 |
| `src/mtc_description` | S3、腕部 D435i、锁扣与电池运行模型 |
| `src/mtc_simulation` | Gazebo 场景、真值、锁扣与逐步接触监测 |
| `scripts` | 构建、启动、基础任务流程、录像和验证 |
| `config`、`docs` | 仿真假设、接口和五人分工 |

`scripts/demo.py` 已有基础阶段流程、顺序调度、执行反馈和异常终止；SJM 在此基础上负责完善任务管理和恢复。感知、标定及其他未实现的空包已删除，职责保留在 [分工表](docs/team/OWNERS.md)。

## 构建与运行

系统依赖：Ubuntu 22.04、ROS 2 Humble/MoveIt 2、Gazebo Harmonic 开发库、配套 `ros_gz`、ros2_control、realsense2_description、colcon、C++17、Python numpy/PyYAML/Pillow、ffmpeg；录像需 `fonts-noto-cjk` 或 `fonts-droid-fallback`。

```bash
bash scripts/build.sh
bash scripts/run_demo.sh --task basic --order C --verify
bash scripts/run_demo.sh --task sequence --order CAB
```

不录像可加 `--no-video`。构建会下载固定版本的控制库并应用本项目补丁，不修改系统 ROS。模型已提交，无需 CAD Python 环境；运行时自动解析项目路径，生成文件、日志和录像都在 Git 忽略的目录内。

构建执行轨迹、短运动与控制器插值检查。若系统 Python 已有 `pybullet==3.2.7`，还会执行动力学交叉检查；否则明确提示跳过。可单独运行：

```bash
source scripts/env.sh
python3 scripts/check_dynamics.py
python3 scripts/check_sensors.py
```

接口见 [规划接口](docs/architecture/INTERFACES.md)，模型来源见 [第三方说明](docs/MODEL_SOURCES.md)。

## 结果边界

输入使用 Gazebo ground truth，未实现真实感知和标定；尚无真机验证。锁扣使用位置/轴向条件约束下的固定连接，不能验证真实材料强度。轨迹选择候选中最快的通过验证方案，不保证全局最优；碰撞检查采用有限采样。历史演示和原始证据在独立归档仓库中。
