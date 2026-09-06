# mtc_hardware

负责人：SJM。状态：为后续实机预留，尚未实现或连接硬件。

封装真实 AUBO S3 控制器/SDK、夹爪驱动和 RealSense 启动，尽量保持仿真与实机上层接口一致。官方驱动参考 [AuboRobot/aubo_ros2_driver](https://github.com/AuboRobot/aubo_ros2_driver)。

预留 `config/`、`launch/`、`adapters/`。本阶段默认 simulation；真实机器人地址、控制器版本、标定参数尚未提供，配置不填写猜测的真实连接地址。实机接入范围取决于 Q1。
