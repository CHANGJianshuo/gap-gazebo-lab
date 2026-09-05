# S3 与末端模型

团队负责人仍为 SJM，韩明赫复核安装参数。本次规划演示已实现组合模型，生成脚本为 `scripts/prepare_assets.py`。

`urdf/s3_latch.urdf` 使用 package URI，供 MoveIt 加载；`s3_latch_resolved.urdf` 为 Gazebo 解析了本机资源路径。原始 AUBO 资料在 `third_party/aubo_description/`，未改动。

`meshes/battery.stl` 来自用户 STEP，毫米转换为米并转到把手向上的姿态，外形 70×80×80 mm；质量和惯量假设见 `config/battery.json`。半圆锁扣为参数化概念设计，不能视为完成制造图纸。腕部 D435i 复用官方内部 TF 和外壳模型，并提供 RGB-D、IMU 仿真。
