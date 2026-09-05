# 常建烁 · 运动规划

已实现并在 Gazebo 执行抓放。入口 [planner.cpp](src/planner.cpp)，接口 [PlanMotion](../mtc_interfaces/action/PlanMotion.action)。生产模块返回轨迹，不选择比赛任务、不执行夹爪状态机。

- [trajectory.hpp](include/mtc_motion_planning/trajectory.hpp)：Ruckig、五次 B 样条、时间缩放、采样与插值。
- [dynamics.hpp](include/mtc_motion_planning/dynamics.hpp)：包含相机、锁扣和电池的全树逆动力学。
- `config/`：实际 SRDF、KDL、关节限制和控制器参数。
- `test/`：轨迹导数、实际控制器插值及独立动力学交叉核对工具。

运行 `source scripts/env.sh` 后使用 `python3 scripts/launch_planner.py --task sequence`。必须同时有仿真状态来源。完整启动及视频命令见 [项目 README](../../README.md)。

选型理由、优化边界和零基础解释见 [实际方案](../../docs/architecture/MOTION_PLANNING_PLAN.md)；SJM 对接见 [接口说明](../../docs/architecture/INTERFACES.md)。
