# 运动规划接口 · 常建烁 → SJM

当前实现使用 Gazebo 真值。任务选择、状态机、重试、夹爪操作和轨迹执行由 SJM 的正式模块负责；`scripts/demo.py` 只是可运行的对接示例。未来感知和标定的团队约定见 [视觉模式草案](INTERFACES_VISION_DRAFT.md)。

## 输入

ROS action：`/motion_planning/plan`，类型 `mtc_interfaces/action/PlanMotion`。

| 字段 | 含义 |
| --- | --- |
| `start_state` | 可空，使用最新 `/joint_states`；当前演示各段从静止状态出发 |
| `target_pose` | TCP 的位置和朝向；支持 `world` 或 `base_link`，单位 m，四元数有效 |
| `mode` | `free` 自由搬运、`cartesian` 末端直线、`joint` 指定六关节目标 |
| `goal_state` | 仅 `joint` 模式使用；填写所有六个关节 |
| `touching_object` | 允许锁扣接触的电池 ID；抬升/落桌时为已连接电池开放必要桌面接触 |
| `velocity_scaling` / `acceleration_scaling` | 正数且不超过 1；0 使用配置默认值，jerk 随加速度比例缩放 |
| `planning_timeout` | 候选搜索预算，秒；最终验证及部分候选计算可能额外耗时，不是硬实时期限 |
| `seed` | 外层 IK 候选初值种子；KDL/OMPL 与计算时间预算仍可能导致运行差异 |

输入位置点还不足以抓取：必须补齐末端朝向、TCP 定义、当前关节状态、障碍物和已夹住的负载。演示将电池把手中心和轴向从 STEP 局部坐标转换到 Gazebo 世界。

自动订阅：`/joint_states`、`/simulation/ground_truth`、`/simulation/latch_state`。当前关节和真值超过 2 秒墙钟时间未更新会拒绝规划。同一时刻只接受一个规划目标；支持取消。

坐标：世界地面 z=0，桌面 z=0.65 m；`base_link` 位于 `[0,0,0.65]`。底图平面 +x 指向 PDF 上方、+y 指向 PDF 左方。固定模型实际 TF 为 `world → base_link → … → wrist3_Link → latch_mount → tcp`，相机保留官方内部光学 TF 树。

## 输出

| 字段 | 含义 |
| --- | --- |
| `success`、`reason` | 必须检查；失败结果禁止执行 |
| `trajectory` | `moveit_msgs/RobotTrajectory`，包含关节名及各点 q、速度、加速度、力矩前馈、`time_from_start` |
| `duration` | 轨迹预计执行时间，仿真秒 |
| `planning_time` | 规划计算墙钟秒 |
| `minimum_clearance` | FCL 按允许接触矩阵计算的最小非豁免距离；不能解释成把手接触间隙 |
| `maximum_torque_ratio` | 逆动力学预测峰值 / URDF 电机力矩限值 |
| `diagnostics_json` | 候选时长、拒绝原因、采样分辨率、载荷和插值检查结果 |

固定关节顺序：`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`。不要把 `/joint_states` 的数组顺序当成此顺序，应按名字匹配。

IK 解是轨迹最后一点的关节位置，通常不需要另一条“IK 结果”执行指令。SJM 将成功结果中的 `joint_trajectory` 发送给 `/arm_controller/follow_joint_trajectory`；动作成功后检查控制反馈。参考实现是 `scripts/demo.py` 的 `plan()` 和 `execute()`。

## 控制与锁扣

本工程的 Humble JTC 是带补丁的独立 overlay：位置/速度/加速度按五次插值，力矩前馈按线性插值，叠加 PID 后由 Gazebo 硬件插件按电机力矩限幅。系统原版 JTC 2.53.1 会拒绝 effort 字段，不能遗漏 `source scripts/env.sh`。

锁扣两个关节使用 `/latch_controller/follow_joint_trajectory`：0 m 闭合，0.022 m 张开。随后调用 `/simulation/latch` 的 `close=true` 才建立受位置与轴向条件约束的机械连接；释放用 `close=false`，再张开并向上撤离。场景会把已连接电池从世界障碍物变为随 TCP 运动的负载，并加入惯量。

`/simulation/contact_report` 汇总每个物理步的接触，区分电池落桌、锁扣接触与非预期接触。报告用于验收，不是实机碰撞急停替代品。
