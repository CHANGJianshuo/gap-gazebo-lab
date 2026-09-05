# 已实现接口

团队接口负责人：SJM。当前运动规划演示实现了 `action/PlanMotion.action` 和 `srv/Latch.srv`，通过 rosidl 生成 C++ / Python 类型。

消息逐字段解释、坐标和控制器要求见 [接口文档](../../docs/architecture/INTERFACES.md)。未来正式任务、感知与标定消息按团队协议补充，避免将仿真真值接口误用为实际感知算法。
