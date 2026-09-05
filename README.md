# 常建烁 · AUBO S3 运动规划演示

ROS 2 Humble + MoveIt 2 + Gazebo Harmonic。使用官方 S3 模型、腕部 D435i、原尺寸比赛底图、用户提供的电池 STEP 和两片半圆锁扣，从仿真真值生成并执行完整抓放轨迹。

[打开制作过程与演示网页](docs/progress/index.html) · [实际验证记录](docs/VALIDATION.md) · [规划原理与方案](docs/architecture/MOTION_PLANNING_PLAN.md) · [给 SJM 的接口说明](docs/architecture/INTERFACES.md)

当前分支 `feat/motion-planning-v2` 转向 **GaP（Graph-as-Policy）在 S3 上的应用研究**：[研究入口](docs/research/gap/README.md) · [论文全文 Markdown](docs/research/gap/paper/gap_2607.05369v1.md) · [中文阅读笔记](docs/research/gap/READING_NOTES.md) · [接入与实验方案](docs/research/gap/S3_RESEARCH_PLAN.md)。目前完成论文整理和方案，尚未运行 GaP；以下演示结果属于保存在 `main` 与 `v0.1.0-sim-demo` 的运动规划基线。

详细通俗技术报告：[网页阅读](docs/reports/motion_planning_report.html) · [19 页 PDF](docs/reports/motion_planning_report.pdf) · [Markdown 原稿](docs/reports/motion_planning_report.md)。包含 IK、B 样条与 Ruckig、动力学和控制、实际规划案例、最终数据及团队对接。

最终录像：[序列 CAB](data/videos/sequence_demo.mp4) · [基础 C→T0](data/videos/basic_demo_final.mp4)。两次最终运行及接口检查均通过，详细数值见验收记录。

网页可直接打开；若浏览器限制本地视频或 JSON，可在项目根目录运行 `python3 -m http.server 8765 --bind 127.0.0.1`，访问 `http://127.0.0.1:8765/docs/progress/`。

## 运行

本机已完成构建。运行时关闭之前手动启动的本项目仿真，避免同一 ROS domain 内出现重复控制器。

```bash
bash scripts/run_demo.sh --task sequence --order CAB
bash scripts/run_demo.sh --task basic --order C
```

每次创建独立的 `data/runs/<运行编号>/` 和 `data/videos/<运行编号>.mp4`，包含轨迹、候选方案、关节跟踪 CSV、真值轨迹、接触记录、日志和验收结果。视频来自 Gazebo 相机，25 fps，按传感器的仿真时间播放；传输丢帧会补持上一帧并记录数量。实际计算机耗时单独记录。

重新生成资产、构建并进行核心检查：

```bash
bash scripts/build.sh
```

系统依赖：Ubuntu 22.04、ROS 2 Humble / MoveIt 2、Gazebo Harmonic 开发库 `libgz-sim8-dev`、与 Harmonic 配套的 `ros_gz`、ros2_control、realsense2_description、colcon、C++17 编译器、Python venv、ffmpeg、中文字体。`build.sh` 固定下载两项控制库源码，创建独立 CAD Python 环境，不修改 `/opt/ros`。

分开启动以方便调试：

```bash
source scripts/env.sh
python3 scripts/launch_sim.py --task sequence
# 另一个同样 source 的终端
python3 scripts/launch_planner.py --task sequence
# 第三个终端
python3 scripts/demo.py --task sequence --order CAB
```

## 实现内容

| 位置 | 内容 |
| --- | --- |
| `src/mtc_motion_planning/` | 常建烁的核心：多初值 IK、Ruckig、五次 B 样条、RRT-Connect 后备、全模型碰撞与力矩验证、ROS action |
| `src/mtc_interfaces/` | 已实现 `PlanMotion.action`、`Latch.srv` |
| `src/mtc_description/` | 官方 S3、相机、参数化锁扣、STEP 派生网格及质量参数 |
| `src/mtc_simulation/` | 两张原始底图世界、Gazebo 真值、受位置约束的锁扣、每个物理步的接触监测 |
| `scripts/demo.py` | 集成测试用抓放流程；正式任务状态机和执行层仍由 SJM 负责 |
| `scripts/run_demo.sh` | 启动、录像、验收、日志归档与本次进程清理 |
| `docs/progress/` | 持续更新的简明中文教学网页 |
| `third_party/` / `vendor_ws/` | 固定版本上游模型及控制库；补丁保存在 `scripts/*.patch` |

其他成员的感知、标定、正式调度等目录仍是团队接口预留，并未用空节点冒充完成。原始 PDF 和 STEP 均保留原位。

## 适用范围

当前是**理想输入的运动规划仿真**。相机真实产生 RGB、深度、内参和 IMU，但规划读取 Gazebo 真值。电池质量 0.4 kg、摩擦系数 0.8、加速度/jerk 限值、相机与锁扣安装参数是明确记录的仿真假设。

锁扣闭合且 TCP 距把手中心不超过 2.5 mm、轴向匹配后，Gazebo 建立固定机械连接；不移动电池去迎合夹爪。该模型验证搬运与释放过程，不验证锁扣材料强度、间隙冲击和真实承载能力。

算法选择候选中**最快且通过全部检查**的轨迹，不宣称求出了带障碍、带动力学约束的全局最优解。有限步长的 FCL 检查与实际物理接触记录分别保留。

底图内圈直径 80 mm，电池平放外包尺寸 70×80 mm，不能完全包含于内圈；本演示检查目标中心放置，不声称获得内圈满分。演示颜色次序由 `--order` 指定，示例 CAB 不是替赛事指定口令。
