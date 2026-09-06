# 模型来源与修改

AUBO S3 来源：AuboRobot/aubo_description，固定提交 `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`。项目保留 7 个连杆的视觉/碰撞网格；组合 URDF 增加相机、锁扣、控制和仿真标签。上游 README、package.xml 的 BSD 声明和源文件校验清单保存在 `third_party/aubo_description/`，不重新指定第三方许可。完整原件在 GaP 仓库 `legacy/meituan_challenge/third_party`。

D435i 来源：realsenseai/realsense-ros 的 realsense2_description，原环境版本 4.57.7，Apache-2.0；运行网格来自安装的 ROS 包。相机惯量和安装参数为仿真假设。

电池网格由用户提供的 STEP 转换，锁扣为参数化概念设计。底图纹理来自用户比赛资料，保持原尺度。原始文件、CAD 生成脚本、完整模型说明及版本记录均保存在 GaP 仓库归档。
