# 原项目完整归档

`meituan_challenge/` 完整保留原仓库 `feat/motion-planning-v2` 分支精简前的 Git 跟踪文件，包括核心代码、研究资料、论文、报告、录像和运行记录。该目录是历史快照，不随精简版更新。

完整提交历史另存为本仓库分支：

- `archive/meituan-main`：精简前 main。
- `archive/meituan-core`：精简后核心代码，包含移除 CAD 构建依赖及可迁移资源路径的改动。
- `archive/meituan-motion-planning-v2`：本地开发分支，包含此前尚未推送的提交。
- `archive/meituan-motion-planning-v2-remote`：原远端开发分支。
- `archive/meituan-backup-wsl-20260905`：原远端备份分支（包括额外的论文整理脚本）；相对于开发分支的文件差异另存于 `branch_variants/backup-wsl-20260905/`。

原版本标签保存为 `archive/meituan-v0.1.0-sim-demo`。

本机 `/home/chang/gap/.local_archive/meituan_challenge/` 另有精简前完整工作目录（含 Git、环境、构建缓存和未跟踪产物），不上传可重建的缓存和环境。

历史记录中的绝对路径保留原样以保护证据；若运行本快照，需重新生成模型路径和构建环境。当前 GaP 开发入口是仓库根目录下的 `gazebo_lab/`。
