# mtc_perception

负责人：孙浩然。状态：目录骨架，感知节点尚未实现。

输入腕部 RGB、depth、CameraInfo；输出相机光学坐标系中带时间戳的颜色、三维位姿、几何、抓取候选和质量。先使用颜色/轮廓/深度基线，必须通过桌面高度排除底图印刷颜色，并支持抓取后、放置后视觉验证。

预留 `mtc_perception/`、`config/`、`launch/`、`test/`，建议 Python + OpenCV + ROS 2。由 calibration 负责向基座坐标转换，不读取 Gazebo 物体真实位姿。验收记录颜色混淆、无效深度、位姿抖动及遮挡下拒绝错误输出的情况。
