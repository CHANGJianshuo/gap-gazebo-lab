# AUBO S3 官方模型来源

上游：[AuboRobot/aubo_description](https://github.com/AuboRobot/aubo_description)

固定 commit：`47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`（上游提交时间 2026-03-13）。

`source_manifest.json` 列出 18 个所需原文件及 Git blob SHA-1，共约 3.73 MB；`scripts/fetch_aubo_s3.py` 校验下载内容。`upstream/` 保存未修改原件。目录包含：

- `urdf/aubo_S3.urdf`：S3 专用几何、惯量和六个转动关节。
- `urdf/aubo_S3.srdf`：原始语义模型，组名 `manipulator_S3`。
- `meshes/aubo_S3/visual/link0.DAE` 至 `link6.DAE`。
- `meshes/aubo_S3/collision/link0.STL` 至 `link6.STL`。
- 上游 `README.md`、`package.xml` 作为来源和许可证元数据。

S3_10 和 S3_T0 的 URDF 在此上游版本是指向 `aubo_S3.urdf` 的别名。本项目下载真正包含 XML 的 canonical 文件；大小写为 `S3`，不存在的 `aubo_s3.urdf` 不可混用。

原始关节：`shoulder_joint`、`upperArm_joint`、`foreArm_joint`、`wrist1_joint`、`wrist2_joint`、`wrist3_joint`。原模型有 7 个机械连杆及 `world`，通过 `world_joint` 固定。接入项目时需要在组合层处理世界关系、相机、夹具、ros2_control 和厂商扩展标签；原模型不等于已完成 Gazebo 控制适配。

原 URDF 使用 `package://aubo_description/...`。这是保留上游路径的源文件子集，不是完整可构建上游包；实现时需在项目描述包中提供资源解析和组合层，不直接构建此目录。`third_party/COLCON_IGNORE` 防止被 colcon 误发现。

上游 package.xml 声明 `BSD`，此次树中未发现独立 LICENSE 文件，原有声明完整保留。报告中应记录采用的模型、版本及修改；不为第三方模型重新指定许可证。

D435i 外形及内部坐标系来自已安装的 `/opt/ros/humble/share/realsense2_description/urdf/_d435i.urdf.xacro`，包版本 4.57.7，上游为 [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros/tree/ros2-master/realsense2_description)，文件声明 Apache-2.0。它复用 D435 机壳网格并加入 IMU frame；还需要 Gazebo 传感器与桥接才能产生仿真数据。其机壳惯量在上游明确标为不可靠，动力学需要在组合层核对，不直接当作实测参数。
