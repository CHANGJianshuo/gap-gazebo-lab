# 初赛接口草案

2026-09-05 范围更新：用户要求优先完整实现常建烁的运动规划，以 Gazebo ground truth 提供理想输入。新增规划演示模式明确允许读取真实关节、电池位姿及环境；该模式不评估感知与标定，也不冒充使用腕部视觉的整场比赛系统。正式任务调度仍归 SJM。制作过程见 [持续更新的 HTML](https://github.com/CHANGJianshuo/gap-gazebo-lab/blob/main/legacy/meituan_challenge/docs/progress/index.html)。下文原有视觉模式的真值隔离约定仍适用于未来视觉比赛 profile。

以下为包实现前的共同约定。消息文件和节点尚未实现；任务范围、物理参数和评分歧义见 [待确认问题](https://github.com/CHANGJianshuo/gap-gazebo-lab/blob/main/legacy/meituan_challenge/docs/OPEN_QUESTIONS.md)。

## 坐标与时间

统一使用 SI：m、rad、s。所有位姿携带 `header.frame_id` 与采样时间，不在视觉或规划代码中散落 mm 补偿。所有仿真节点 `use_sim_time=true`，使用同一 `/clock`。

```text
world
└── board                         底图/桌面坐标
    └── base_link                 固定安装关系
        └── shoulder_Link
            └── upperArm_Link
                └── foreArm_Link
                    └── wrist1_Link
                        └── wrist2_Link
                            └── wrist3_Link
                                ├── tool_mount → gripper → tcp
                                └── camera_mount → camera_link
                                    ├── camera_color_optical_frame
                                    ├── camera_depth_optical_frame
                                    └── IMU 相关 frame
```

示意图省略 RealSense 的中间内部 frame；正式模型应复用其官方 Xacro 中的完整树。S3 原 URDF 自带 `world_joint`，组合模型时只保留一个有效世界固定关系，避免重复根节点和 TF 多父节点。

`T_base_camera(t)=T_base_wrist(t)·T_wrist_camera`。使用图像采集时刻的关节状态与 TF 查询，拒绝拿“最新 TF”转换旧图像。先采用到观察位、静止采样的流程，再按实际延迟决定是否需要运动中感知。

## 模块数据约定

| 接口 | 生产方 → 消费方 | 建议字段/语义 |
| --- | --- | --- |
| RGB、深度、CameraInfo | simulation/hardware → perception | 时间同步、光学坐标、编码、内参与有效深度范围 |
| Camera battery observations | perception → calibration | ID、颜色、PoseWithCovariance、尺寸、置信度、抓取候选、源图像时间 |
| Base battery observations | calibration → task/planning | 同一观测转换到 base_link；保留协方差、时间戳和失效原因 |
| Competition task | 输入 → task_manager | 模式 basic/sequence、任务编号、颜色口令、目标位置、有界重试配置 |
| Pick/place request | task_manager → manipulation（均为 SJM） | 目标 ID、电池观测、放置要求、抓取候选、任务期限 |
| Motion planning request | manipulation（SJM）→ motion_planning（常建烁） | 目标 TCP 的 PoseStamped、起始关节状态、场景版本、运动方式、位姿容差、速度/加速度限制、规划超时 |
| Motion planning result | motion_planning（常建烁）→ manipulation（SJM） | 带关节名和时间的 RobotTrajectory、成功/失败状态、失败原因、规划耗时；可选返回目标关节解 |
| Execution result | manipulation → task_manager | 阶段、成功/失败码、夹爪/关节反馈、可否恢复 |
| Run event | 各模块 → evaluation | run_id、sim_time、wall_time、任务阶段、事件、版本与配置哈希 |

跨节点操作优先 ROS 2 action，支持反馈、取消和明确结果；持续数据用 topic，短查询用 service。具体 `.msg/.srv/.action` 在第一阶段实现并用示例消息验收。

## 状态机与执行边界

唯一 Task Manager 由 SJM 负责：`INIT → WAIT_COMMAND → OBSERVE → DETECT → SELECT → PLAN → APPROACH → GRASP → VERIFY_GRASP → TRANSFER → PLACE → VERIFY_PLACE → NEXT/FINISH`。

阶段有超时、失败码和有限次数重试。感知丢失重新观察，规划失败更换可达候选，抓取失败先撤离再重试；不可恢复时取消动作并停在可解释状态。任务运行中不接受第二个互相冲突的执行请求。

Manipulation 提供靠近、抓取、夹爪闭合、抬升、转运、下放、释放、撤退等动作；moveit planning scene 的 attached object 只用于碰撞规划，不等同于 Gazebo 中已真实抓住物体。

常建烁负责从运动请求生成关节轨迹。电池中心位置经过夹具/TCP 几何和抓取方向转换，才成为目标 TCP 位姿；建议由 SJM 的动作层完成该转换，规划层负责 IK、路径和运动约束验证。模型、当前关节状态及场景作为模块配置/状态输入，不要求每次复制进同一条请求。详见 [运动规划模块输入输出](../../src/mtc_motion_planning/README.md)。

夹爪确定后建立接触与摩擦模型。任何为诊断加入的理想吸附/附着机制必须单独标识，不能用它掩盖夹持失败并报告物理抓取成功。

## 评测边界

算法链只读取腕部相机、TF、关节及夹爪反馈、任务输入、公开的底图几何。Gazebo 电池真实位姿、碰撞接触和随机种子仅供独立评测与调试，不作为目标观测直接传给算法。

T0 与序列评分分别依据原规则；序列完整包含判定使用电池投影轮廓与圆圈边界，不能只看中心是否在内。边界线宽、允许接触边线、稳定速度阈值和长停滞阈值需记录其来源；没有官方数值的项目只设明确标注的项目验收参数。

第三视角只用于演示/录像，不能成为未声明的第二台算法相机。记录总仿真耗时、墙钟耗时、实时率，录屏不倍速。
