# GaP × AUBO S3 研究入口

本目录属于 `feat/motion-planning-v2`。当前完成的是**论文归档、全文转换、官方接口核对和研究方案**；尚未安装 GaP 运行栈、生成策略图或运行 GaP 仿真实验。

| 先看什么 | 文件 |
| --- | --- |
| 通俗理解论文、判断它能解决什么 | [中文阅读笔记](READING_NOTES.md) |
| 怎样接入 S3、分阶段实验、如何判断有无提升 | [S3 研究方案](S3_RESEARCH_PLAN.md) |
| 论文全文，包含参考文献和附录 | [Markdown](paper/gap_2607.05369v1.md) · [52 页 PDF](paper/gap_2607.05369v1.pdf) |
| 原始来源、下载校验和、转换检查 | [来源记录](paper/source_manifest.json) · [转换质量](paper/conversion_quality.json) |
| 可检索的 PDF 原始提取文本 | [按页文本](paper/pdf_text.txt) |

论文为 Kaiyuan Chen 等人的 **GaP: A Graph-as-Policy Multi-Agent Self-Learning Harness For Variational Automation Tasks**，2026-07-06 提交，固定阅读版本 `arXiv:2607.05369v1`。检索与接口核对日期：2026-09-05。[arXiv 原页](https://arxiv.org/abs/2607.05369v1) · [作者项目页](https://graph-robots.github.io/gap/)。

原有运动规划成果保存在 `main` 与 `v0.1.0-sim-demo`，基线提交 `8f204d6e8c3f85d5dc602ac1afd5f865761feffd`。研究方案引用原有实测数据时明确标注为基线；当前没有 GaP 成功率或加速结果。

## 目录与来源

```text
docs/research/gap/
├── README.md                     阅读入口与来源说明
├── READING_NOTES.md              中文原理、实验结果及适用边界
├── S3_RESEARCH_PLAN.md           接口方案与分阶段实验设计
├── paper/
│   ├── gap_2607.05369v1.pdf      作者原始 PDF
│   ├── gap_2607.05369v1.md       同版本全文转换，非摘要
│   ├── arxiv_2607.05369v1.html   保留公式与原始代码的来源
│   ├── arxiv_abstract.html      版本、作者及许可来源
│   ├── images/                  5 张 PNG、5 张 SVG，均为作者原图
│   ├── listings/                附录 4 份精确恢复的提示词/代码
│   ├── pdf_text.txt             PDF 页码辅助检索
│   ├── source_manifest.json     下载 URL、文件大小、SHA-256
│   └── conversion_quality.json  结构覆盖与表格、代码校验
└── upstream/
    ├── graph-as-policy/         官方代码和文档的选定阅读快照
    ├── open-robot-skills/       官方技能库说明与依赖快照
    ├── *-manifest.json          各快照的固定提交、来源、SHA-256
    └── *-tree.json              官方仓库目录树，辅助查找适配入口
```

`upstream/` 是**选定文件的只读参考副本**，不是完整可运行的源码安装。官方引擎固定于 [`93e929c9a1668cacba2e8be604e99a7538abbee9`](https://github.com/graph-robots/graph-as-policy/tree/93e929c9a1668cacba2e8be604e99a7538abbee9)；技能库固定于 [`925d78eaf4873c3130c79b9b8f21971ec4e8b887`](https://github.com/graph-robots/open-robot-skills/tree/925d78eaf4873c3130c79b9b8f21971ec4e8b887)。保留两者 Apache-2.0 许可，来源见各自 manifest。后续安装使用独立 checkout 和环境，不把这些摘录加入 ROS 的 Python 搜索路径。

## 如何重新生成 Markdown

在仓库根目录运行：

```bash
# 使用已归档的 PDF、HTML 和图片，离线重新转换并检查。
python3 scripts/prepare_gap_paper.py

# 仅在原始文件或图片缺失时下载；已有文件会校验 SHA-256。
python3 scripts/prepare_gap_paper.py --download
```

转换器依赖 Python 3、BeautifulSoup4、lxml、requests 和 PyMuPDF，本机已具备。它不运行论文里的任何代码。

采用同版本 arXiv HTML 恢复公式和排版，PDF 作为原本保留；共处理 150 处数学表达、54 个标题、4 张结果表、11 处插图引用（10 个独立文件）和 4 份完整原始代码/提示词。HTML 错位的作者同等贡献脚注依据 PDF 第 1 页校正；其他原文错字和科学结论不擅自修改。结构校验通过，另人工查看 PDF 第 1、8、10 页的架构、表格与 ROS 示例。计数检查验证结构覆盖，不代表论文结论已经复现实证。

论文原文及插图依作者指定的 [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) 许可归档和转换，作者、来源及转换说明已写入 Markdown 开头。中文笔记与 S3 方案是本项目的解释和建议，均与作者原文区分。
