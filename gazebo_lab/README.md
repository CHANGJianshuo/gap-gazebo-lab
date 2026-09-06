# GaP × Gazebo：机械臂策略图与反馈修复实验

这里研究的是 **GaP 在机械臂上的应用**，独立于比赛项目。机械臂为 Franka Panda，仿真为 Gazebo Harmonic，底层使用 ROS 2 Humble、MoveIt 2 和 ros2_control。可视化界面直接显示 Gazebo 相机画面及官方 GaP 运行时的节点事件。

## 已实现与复现边界

这不是论文全部实验的等价复现，而是**官方图执行器的 Gazebo 适配 + 独立实现的图外反馈修复循环**。

- 官方 GaP v3 解析、验证、工具调度、执行与 trace：直接使用 `../graph-as-policy`，没有修改上游源码。
- Gazebo/MoveIt 工具适配、世界真值、抓取约束、修复循环、可视化：本目录新增。
- 规则修复：可以完全离线运行，规则确实修改图结构，但不是大模型生成。
- LLM 修复：已提供兼容 Chat Completions 的真实请求入口，需要模型与 API 配置；未配置时界面禁用，不能声称已经验证模型能力。
- 只支持一个受限的越障搬运实验；当前补丁只能在失败的 `carry` 前插入 1–6 个运动节点。不等于通用任务规划器，也没有复现论文多智能体训练、Isaac rehearsal、全部 benchmark 或真实机器人实验。

截至本次下载，官方 [roadmap](https://github.com/graph-robots/graph-as-policy/blob/93e929c9a1668cacba2e8be604e99a7538abbee9/docs/source/developers/roadmap.md) 把 execution-feedback graph repair 和 Isaac rehearsal 列为公开 v1 之后的工作。**图内循环**只是重复执行现有节点；**图外循环**会根据执行结果产生下一版本的图，二者不同。

## 启动

已有依赖及构建产物时：

```bash
cd /home/chang/gap_reproduction/gazebo_lab
source scripts/env.sh
/usr/bin/python3 scripts/launch.py
```

打开 **http://127.0.0.1:9433**。可加入 `--gui` 同时打开 Gazebo 原生窗口。首次启动需要等待控制器和相机就绪。界面依次点击“运行实验”，结束后可以点击“重置场景”再运行。

停止：在启动终端按 `Ctrl+C`。启动器只管理自己创建的进程组。ROS domain 固定为 84，Gazebo partition 为 `gap_lab`。两个 HTTP 服务仅监听本机：9431 是 ROS 适配器，9433 是实验界面。不要同时启动两份实验。

**“停止执行”是仿真实验的取消功能，不是实体机器人的安全急停。** 控制器取消仍需要传播和制动时间。

## 场景和成功标准

蓝色方块从 `[0.50, -0.24, 0.17]` 米搬到 `[0.50, 0.24, 0.17]` 米。隔板顶面高 `0.38 m`。所有坐标均为 `world` 系，单位为米。

初始图在抬到 `0.25 m` 后试图横移，规划器计算完整笛卡尔路径的可达比例；比例不足 0.999 就拒绝执行整段，部分路径不会偷偷执行。

规则修复读取实际失败信息，在 `carry` 前新增：

```text
low_lift → lift_clearance (z=0.49)
         → transfer_clearance (目标区上方 z=0.49)
         → carry (下降到 z=0.25) → place → release → retreat → verify
```

放置节点在 `z=0.178 m` 松手，方块经过约 8 mm 的物理下落停在桌面。独立验收要求：未被夹持、方块中心距离目标小于 35 mm、连续 0.5 秒位置变化小于 2 mm，以及未记录到指定的非预期接触。图内 `verify` 和图外最终判定使用同一个受保护的评估器，改图器不能修改它。

## 循环究竟如何工作

1. 把 v0 写为真正的 GaP `workflow.json`，通过上游验证器。
2. 官方 `WorkflowExecutor` 按图调用 `gazebo.*` 工具，ROS 适配器执行实际规划和控制。
3. 失败时保存节点、错误码/路径比例、当前物体和末端真值。
4. 规则或 LLM 提出 JSON 补丁；补丁验证器检查字段、目标节点、坐标范围、节点数量和动作类型。
5. 形成 v1，重新调用官方结构验证器，执行器继续运行。
6. 成功或达到最多指定轮数后停止，保存完整结果。

**本演示采用同一物理回合的前缀恢复，不是把机器人偷偷回到初始位置。** v1 中已经成功的相同节点返回本回合缓存结果，界面标为紫色；新增节点和失败节点实际重新执行。缓存不跨实验复用。当前场景静态、单臂、串行，机器人状态由同一适配器持有；若要推广到动态场景，恢复前还需要更完整的世界状态一致性验证。

“重置场景”是用户触发的回合初始化：打开夹爪、机械臂回到高位、重新摆放方块。它会使用 Gazebo 的物体 pose 设置，仅限回合之间，未注册成 GaP 工具，修复器不能调用。

## 各模块做什么

| 文件 | 输入 → 输出 | 职责 |
|---|---|---|
| `scripts/prepare.py` | 上游 Panda 模型 → 本地 URDF、SRDF、控制器、世界文件 | 生成可复建场景，保留来源信息 |
| `src/world.cpp` | Gazebo 实体状态 → `/gap/world`；闭合请求 → 抓取约束 | 发布真值、位置门限抓取、接触报告 |
| `scripts/ros_adapter.py` | `observe/move/gripper/verify` → 真值、实际执行结果或失败 | ROS 3.10 进程；对接 MoveIt、控制器和相机 |
| `scripts/trajectory_math.py` | 轨迹位置/速度/加速度 → 插值极值与时间缩放 | 检查控制器五次多项式插值的速度、加速度、jerk |
| `scripts/server.py` | 图与执行反馈 → 版本图、补丁、trace、最终验收 | Python 3.12 进程；使用官方 GaP 运行时，管理图外循环 |
| `web/index.html`, `web/app.js` | 实验状态、相机图像 → 可交互界面 | 看图、选节点、切版本、查看改图和控制实验 |
| `scripts/launch.py` | 本地配置 → 独立 Gazebo/ROS/Web 进程组 | 启停、日志、端口与进程隔离 |
| `tests/test_repair.py` | 安全/越界补丁、插值样例 → 测试结果 | 检查修复边界与轨迹极值计算 |

GaP 是“下一步做什么”，MoveIt 是“关节怎样运动”。不要让大模型直接生成未经检查的电机指令。这里每个 `move` 都经过 IK/路径规划、碰撞检查，再由 JointTrajectoryController 执行。

## 轨迹与物理模型的限制

- 自由空间接近：KDL IK → OMPL RRTConnect → MoveIt 时间参数化。
- 抓取、越障和放置：带碰撞约束的 Cartesian path → MoveIt 返回的时间参数化轨迹。
- 保留位置、速度、加速度，按 JTC 的五次插值计算每段导数极值，统一放慢时间，使实验臂轨迹的峰值满足 `0.6 rad/s`、`1.2 rad/s²`、`8 rad/s³`。
- 发给控制器之前还检查插值路径上的离散样本（原始轨迹时间间隔最多 20 ms）。这是离散碰撞检查，不能称为连续碰撞证明。拾取开始时仅对 cube/table 的支撑接触允许固定 0.1 mm 数值容差，其他碰撞对保持严格检查，修复器无法改动此设置。
- 当前 arm 使用 `gz_ros2_control` 的位置指令/速度伺服接口，**不是经过标定的真实 Panda 力矩控制器**。惯量来自上游辨识参数，抓取使用有 16 mm TCP 距离和夹爪闭合条件的固定约束，不模拟夹持摩擦极限。
- 接触真值以 20 Hz 报告；非预期接触监测不等价于每个物理子步都无碰撞的证明。
- 此实验优先验证图循环，不宣称运动时间最优或可直接上真机。
- 感知使用仿真真值，没有 DINO/SAM3 等视觉模型，因此 8 GB 显卡可以承载本实验，但这不证明能运行论文完整视觉/策略模型栈。

## 使用真实 LLM 修复

在启动服务的终端中配置，密钥不要写入仓库：

```bash
export GAP_LLM_BASE_URL=https://openrouter.ai/api/v1
export GAP_LLM_MODEL='你的模型标识'
read -rsp 'API key: ' GAP_LLM_API_KEY
export GAP_LLM_API_KEY
source scripts/env.sh
/usr/bin/python3 scripts/launch.py
```

界面选择“LLM 修复”。它会把图、任务几何和真实失败反馈发送给配置的模型接口，响应经过同样的受限补丁验证与实际仿真验证。模型请求失败会明确终止，不会自动降级为规则并冒充模型成功。模型需支持 Chat Completions 和 JSON object 响应格式。日志只保存模型名、用量和图补丁，不记录密钥。

## 结果和检查

- `outputs/runs/<时间>/summary.json`：一次实验的状态和各轮结果。
- `outputs/runs/<时间>/v0/workflow.json`、`v1/workflow.json`：修改前后图。
- 每轮的 `feedback.json`、`repair.json`、`trace/`：失败事实、补丁、官方 GaP trace。
- `events.jsonl`：节点输入输出和修复事件。
- `outputs/dashboard.png`、`outputs/ui-check.json`：实际浏览器截图与检查。
- `outputs/video/`：实际运行的浏览器录像；`scripts/ui_capture.py --run` 可重新录制。
- `outputs/live/*.log`：当前仿真/ROS/网页日志。

测试：

```bash
cd /home/chang/gap_reproduction
PYTHONPATH=graph-as-policy .venv-audit/bin/python -m pytest gazebo_lab/tests -q
```

首次取源码可运行 `python3 scripts/fetch_dependencies.py`（按锁定 commit 拉取，不自动升级系统）。它需要已安装 ROS/Gazebo 开发依赖；Python 3.12 的 GaP 审计环境仍使用上一级项目既有的 `.venv-audit`。

构建与版本锁定见 `scripts/build.sh`、`config/sources.json`、`config/dependencies.json`。目前通过的只是本实验的证据；后续研究应比较无修复/规则/LLM，多次随机场景的成功率、修复轮数、总耗时和模型成本，避免把一个演示当成泛化结论。
