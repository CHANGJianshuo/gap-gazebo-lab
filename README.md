# GaP 独立复现与开源完整性核查

独立仓库：<https://github.com/CHANGJianshuo/gap-gazebo-lab>。本仓库收录 GaP 调研、论文归档和 Gazebo 原型，与 `meituan_challenge` 的比赛 `main` 分支分开维护。

本地目录为 `/home/chang/gap`。原项目所有源码及开发分支内容保存在 [legacy](legacy/README.md)，完整 Git 历史保存在 `archive/meituan-*` 分支。精简后的比赛核心代码继续在原仓库维护。

**当前是研究原型。** 历史版本已完成一次规则改图的 Gazebo 抓放闭环，证据见 [演示归档](artifacts/README.md)。后续轨迹插值检查和启动清理修改尚未完成最终端到端验收，最近重启遇到控制器启动超时；不能把历史录像视为当前 HEAD 稳定可复现的证明。真实 LLM 修复未运行，论文完整自学习与基准实验未复现。

获取仓库和固定版本的官方源码：

```bash
git clone --recurse-submodules https://github.com/CHANGJianshuo/gap-gazebo-lab.git
cd gap-gazebo-lab
```

官方源码以 submodule 引用；Python 环境、模型权重、Gazebo 构建产物和临时运行目录不提交。Gazebo 模型/控制依赖按 `gazebo_lab/config/dependencies.json` 获取。论文 PDF、Markdown 及原始来源记录见 [research/paper](research/paper)。

这里只研究 GaP 在机械臂上的公开实现。核心结论：**公开的图生成、验证和抓放框架可以开展复现；论文完整的仿真排练与执行反馈改图系统没有随当前公开版本发布。**

阅读 [开源与复现核查](reports/OPENNESS_AND_REPRODUCTION.md)。源码和本机环境见 [版本记录](reports/source_environment_audit.json)。

```text
gap/
├── graph-as-policy/        官方完整 Git checkout，含递归子模块
├── open-robot-skills/      官方技能库，和主仓库并列
├── .venv-audit/            独立的核心功能测试环境
├── scripts/               本地复查脚本
├── outputs/authored_graph/ 官方 build_a_graph 示例构建产物
├── logs/                  原始测试和构建日志
└── reports/               开源核查、固定版本与依赖、测试结果
```

主仓库固定提交 `93e929c9a1668cacba2e8be604e99a7538abbee9`，技能库固定提交 `925d78eaf4873c3130c79b9b8f21971ec4e8b887`。当前 checkout 保留官方源码，不做机器人型号替换和物理参数修改。

## 当前环境能验证什么

`.venv-audit` 只安装核心测试所需依赖，以源码路径加载 `gap`，安装 `gap-core`。这是明确裁剪的审计环境，不是官方完整的 `uv sync` 安装，也不冒充其依赖锁定下的论文复现环境。

已构建 `build_a_graph` 的四子图抓放示例，保留官方技能脚本和检查点。仅生成与验证图，没有执行其视觉模型和机器人动作。`skills check` 即使显示 PASS，也可能同时提示工具专属环境缺失；这不能当成模型推理已经可用。

## 重新进行核心检查

以下命令在本目录执行，依赖 `uv`：

```bash
uv venv --python 3.12 .venv-audit
uv pip install --python .venv-audit/bin/python \
  -r reports/requirements-audit.lock -e graph-as-policy/gap-core
.venv-audit/bin/python scripts/run_core_checks.py
```

已有 `.venv-audit` 时只需最后一行。检查脚本运行官方 runtime、builder、tools、skills、agent 五组测试，以及官方图构建和技能格式/导入检查；不会调用真实模型服务或连接机械臂。结果写入 `logs/` 和 `reports/core-checks.json`。测试包含官方预设响应与模拟对象，不能解释为真实 LLM 或真实抓放验证。

完整公开版仿真还需补齐 CUDA/模型环境、SAM3 权重访问和 LLM/VLM 服务，并按官方示例运行。完整论文自学习部分则需要作者补充研究代码，或由我们依据论文独立实现并明确记录差异。

## Gazebo 策略图实验

新增独立的 [Gazebo 实验室](gazebo_lab/README.md)：Franka Panda、MoveIt、官方 GaP 运行时、实时图界面，以及自建的受限执行反馈修复循环。离线规则修复与真实 LLM 接口明确区分。启动后访问 http://127.0.0.1:9433 。
