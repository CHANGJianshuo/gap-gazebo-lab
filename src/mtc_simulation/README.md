# Gazebo 规划测试环境

团队负责人仍为张叔阳。本次规划演示已实现 `worlds/basic.sdf`、`worlds/sequence.sdf`、原底图纹理和道具、`SimulationSystem` 真值与锁扣插件。

底图从 PDF 矢量读取坐标，保持原比例。世界桌面 z=0.65 m；全部电池保持原 STEP 尺寸。模型与场景由 `scripts/prepare_assets.py` 生成。

插件发布 `/simulation/ground_truth`、`/simulation/latch_state`、`/simulation/contact_report`，提供 `/simulation/latch`。闭合锁扣并通过把手位置与轴向检查后创建固定连接，释放后由重力和桌面接触决定电池运动。

每个物理步检查接触数据。电池与桌面、锁扣与把手的必要接触单独记录；锁扣接触电池主体以及其他非预期接触会导致演示验收失败。接触记录属于仿真验收，不是实机安全控制。
