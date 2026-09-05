# 技术报告的维护

阅读 [HTML](motion_planning_report.html)、[PDF](motion_planning_report.pdf) 或 [Markdown](motion_planning_report.md)。HTML 的图片已经嵌入，可以离线阅读；视频和源码链接指向项目文件。PDF 包含 14 个章节书签。

修改正文时编辑 `motion_planning_report.md`，版式在 `report.css`。带 `BEGIN / END` 标记的数据表会在构建时从 `sequence_demo` 和 `basic_demo_final` 的最终 JSON 重新生成，不应手工填写。不要把早期调试运行替换为最终证据。

在项目根目录执行：

```bash
python3 scripts/build_technical_report.py --pdf
```

文档环境依赖 Python Markdown、Matplotlib、Playwright / Chromium 和 PyMuPDF；当前本机已具备。无需启动 ROS 或 Gazebo。省略 `--pdf` 只更新 Markdown 数据表、图和 HTML。

`assets/report_facts.json` 记录源文件哈希与派生指标，`data/validation/technical_report.json` 保存交付时的链接、页面、交互和 PDF 检查结果。正文仍需人工核对，生成图表并不自动保证所有文字结论正确。
