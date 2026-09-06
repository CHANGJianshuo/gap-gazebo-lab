# 第三方来源与许可

- `graph-as-policy`、`open-robot-skills` 是作者官方仓库的固定提交引用，保留各自 LICENSE 和 NOTICE，核心采用 Apache-2.0。递归依赖遵循各自许可，详见 `reports/OPENNESS_AND_REPRODUCTION.md`。
- `outputs/authored_graph` 是官方构建示例产物，脚本来自 `open-robot-skills`，对应许可附于目录内的 LICENSE。
- `research/paper` 归档论文 *GaP: A Graph-as-Policy Multi-Agent Self-Learning Harness For Variational Automation Tasks*，arXiv:2607.05369v1。原作者署名保留在 PDF、HTML 和 Markdown 中；来源声明为 CC BY 4.0，记录见 `source_manifest.json`。Markdown 是由原始页面转换的阅读版本，并非作者原稿。
- Panda 模型和 Gazebo 控制插件由构建脚本按固定版本另行下载，来源见 `gazebo_lab/config/dependencies.json`，不将其许可扩展为整个依赖栈的统一许可。

本项目是独立调研与适配，不是 GaP 作者的官方发布，也不代表论文全部实验已复现。
