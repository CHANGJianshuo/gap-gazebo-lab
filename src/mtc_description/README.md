# S3 与末端模型

负责人：SJM，韩明赫复核安装参数。运行模型已提交，无需 CAD 转换。`scripts/runtime_assets.py` 将模板中的项目路径及 package URI 解析到 `data/runtime`，用于 Gazebo。MoveIt 保留 package URI。

电池外形为 70×80×80 mm，来自用户 STEP；质量与惯量为仿真假设。半圆锁扣是概念模型，腕部 D435i 复用官方外形和内部 TF。来源见 [模型说明](../../docs/MODEL_SOURCES.md)，原始 CAD 和生成脚本在 GaP 仓库完整归档中。
