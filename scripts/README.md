# 常用脚本

| 命令/文件 | 用途 |
| --- | --- |
| `bash scripts/build.sh` | 固定上游源码、应用补丁、转换 CAD、构建和核心检查 |
| `bash scripts/run_demo.sh --task sequence --order CAB` | 连续三块仿真、录像、验证和日志归档 |
| `bash scripts/run_demo.sh --task basic --order C --verify` | 基础任务，再进行不执行机械臂动作的输入拒绝检查 |
| `source scripts/env.sh` | 项目专用 ROS domain、Gazebo partition、Harmonic overlay |
| `launch_sim.py` / `launch_planner.py` | 分别调试环境与规划服务 |
| `demo.py` | SJM 可参考的集成测试调用方式 |
| `capture_frames.py` | 实际传感器录像，依据仿真时间补帧 |
| `check_dynamics.py` / `check_sensors.py` / `check_rejections.py` | 动力学交叉检查、传感器检查、错误输入检查 |
| `update_progress.py` | 同步更新网页 JSON 和直接打开时的嵌入快照 |
| `snapshot_run.py` | 记录配置、源码、二进制哈希和上游版本 |
| `probe_environment.py` / `extract_board_geometry.py` / `fetch_aubo_s3.py` | 原始资料与环境取证 |

上游补丁只写入 `vendor_ws`，不改系统 ROS 安装。相机、锁扣和未提供的质量参数属于仿真假设，见 `config/scenario_assumptions.yaml`。
