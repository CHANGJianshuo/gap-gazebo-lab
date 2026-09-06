# GaP 开源与完整复现核查

核对日期：2026-09-05。对象为作者官方 `graph-as-policy` 和 `open-robot-skills`，聚焦机械臂方法本身。

**结论：可以继续复现公开版的“语言生成技能图 → 验证 → 机械臂抓放 → 评测”流程；当前公开代码不足以原样复现论文完整的自学习系统及全部实验。** 核心工程代码开源，论文系统的若干关键研究组件没有包含在当前发布版中；模型权重、模型服务与硬件又有各自的获取条件。

这修正了上一轮方案中过于乐观的接入判断：仿真排练与执行反馈改图不能当作已经可直接调用的上游功能。作者的 [roadmap](https://github.com/graph-robots/graph-as-policy/blob/93e929c9a1668cacba2e8be604e99a7538abbee9/docs/source/developers/roadmap.md) 明确列出了公开版的裁剪范围。

## 1. 已经下载并验证了什么

| 内容 | 本地位置或结果 |
| --- | --- |
| 官方主仓库，含 Git 历史和远端分支 | `graph-as-policy/`，提交 `93e929c9a1668cacba2e8be604e99a7538abbee9` |
| 官方技能库 | `open-robot-skills/`，提交 `925d78eaf4873c3130c79b9b8f21971ec4e8b887` |
| 直接子模块 | LIBERO-PRO、Variational-Automation-Benchmark、robosuite、robots_realtime |
| 递归子模块 | robots_realtime 下的 i2rt、lerobot_teleoperator_yamactiveleader |
| 官方 core 测试 | **385 passed，2 deselected**；26.71 s；runtime、builder、tools、skills、agent 五组 |
| 官方 `build_a_graph` 示例 | 已生成四个子图的抓放工作流、5 个脚本及检查点；**0 error、0 warning** |
| `skills check` | **18 PASS、0 WARN、0 FAIL**，仅说明本次格式与导入检查通过 |

源码保持官方版本。固定版本与子模块见 [source_environment_audit.json](source_environment_audit.json)；原始结果见 [测试日志](../logs/core-tests.log)、[JUnit XML](core-tests.xml)、[图构建日志](../logs/build-graph.log)、[技能检查日志](../logs/skills-check.log)。

这些是软件验证，**没有运行真实 LLM 生成、视觉模型推理、机器人物理抓放或论文自学习实验**。其中 Agent 单元测试使用官方预设响应；两个真实 LLM 测试被 marker 排除。`skills check` 还明确显示若干工具专属 venv 缺失，PASS 不能被解释为这些模型已经能执行。

第一次核心检查为 380 passed、3 skipped、2 failed、2 deselected；失败源于本审计环境没有安装可选 `google-genai` SDK。按官方 vertex extra 的版本约束补装 `google-genai==1.59.0` 后，五组测试全部通过，原始首次日志也保留为 `core-tests.initial.log`。

## 2. “完全开源”需要分层看

| 层面 | 核查结果 |
| --- | --- |
| GaP 核心工程代码 | 两个官方仓库公开，根 `LICENSE` 为 Apache-2.0 |
| 论文完整研究代码 | 当前公开版缺少内部 Isaac 排练与基于执行失败的自动改图等关键部分 |
| 第三方软件与机器人资产 | 按各自固定版本和许可获取，不能统一视为 GaP 的 Apache-2.0 |
| 模型权重 | 在其他模型仓库获取；SAM3 有访问申请与单独许可，未随 GaP 仓库提供 |
| 生成与视觉推理服务 | 默认示例使用外部 LLM/VLM API；仓库公开客户端代码，不包含这些服务背后的模型训练与权重 |

核心许可依据：[主仓库 LICENSE](https://github.com/graph-robots/graph-as-policy/blob/93e929c9a1668cacba2e8be604e99a7538abbee9/LICENSE) 与 [技能库 LICENSE](https://github.com/graph-robots/open-robot-skills/blob/925d78eaf4873c3130c79b9b8f21971ec4e8b887/LICENSE)。模型访问依据：[SAM3 官方模型页](https://huggingface.co/facebook/sam3)；API 与默认安装需求依据：[官方 README](https://github.com/graph-robots/graph-as-policy/tree/93e929c9a1668cacba2e8be604e99a7538abbee9)。

有两处应如实记录的许可信息：主仓库 `NOTICE.md` 的开头仍写 MIT，与当前根 LICENSE/README 的 Apache-2.0 不一致；`robots_realtime` 固定版本根目录没有 LICENSE，官方 NOTICE 也说明了这一点。因此可以准确地说“两项核心仓库采用 Apache-2.0”，不应扩大成“整个依赖栈及所有权重都按 Apache-2.0 完整开源”。[官方第三方说明](https://github.com/graph-robots/graph-as-policy/blob/93e929c9a1668cacba2e8be604e99a7538abbee9/NOTICE.md)

## 3. 论文能力与公开代码逐项对应

| 能力 | 公开版状态 | 复现含义 |
| --- | --- | --- |
| 多 Agent 从语言生成技能图 | 有 `gap/agent/`、提示词、示例和接口测试 | 配置模型服务后可以做真实生成实验 |
| 图结构与脚本静态检查、生成阶段修复 | 有 | 本轮已验证其中的离线功能 |
| 图执行、分支、数据传递、日志 | 有 `gap/runtime/` | 本轮运行了对应的官方测试 |
| 用仿真真值验证检查点 | 有 | 仍需接上实际仿真；默认 `warn` 不等于失败会终止任务 |
| LIBERO/MuJoCo 抓放与变化场景评测 | 有官方示例、配置、仿真子模块 | 可以作为下一阶段的标准复现目标 |
| Isaac 内部并行仿真排练 | **未包含在 v1** | 无法直接复现论文原排练管线 |
| 执行失败 → 自动分析 → 修改图 → 再实验 | **原研究中的完整循环未移植** | 需要作者补充代码，或依据论文独立实现 |
| 双臂洗箱完整流程 | 当前发布版未提供完整策略流程 | 场景资产存在，不等于策略与实验已齐全 |
| 爆米花长任务与接触式 USB 插入 | 当前 v1 不覆盖完整流程 | 不能以普通抓放示例代替这些论文实验 |
| 实机 Franka | 有控制连接器及示例 | 需要机器人、相机、驱动和标定条件 |
| 实机 UR + ZED | 公开连接器仅提供感知/状态读取 | 并未接出论文 USB 插入需要的运动控制技能 |

主要范围依据：[官方 roadmap](https://github.com/graph-robots/graph-as-policy/blob/93e929c9a1668cacba2e8be604e99a7538abbee9/docs/source/developers/roadmap.md)。UR 的限制同时由 [real.py](https://github.com/graph-robots/graph-as-policy/blob/93e929c9a1668cacba2e8be604e99a7538abbee9/gap/connector/real.py) 的工具注册逻辑确认。

源码还能提供比 README 宣传语更明确的证据：

- [launcher.py](../graph-as-policy/gap/agent/launcher.py) 的模块说明列出了从研究源码中删除的 rehearsal passes 等内容。
- [config.py](../graph-as-policy/gap/agent/config.py) 明确删除了 rehearsal、scene_spec、GRPO 等研究配置。
- [subgraph_runner.py](../graph-as-policy/gap/agent/subgraph_runner.py) 的 `_inject_feedback()` 说明原始 typed rehearsal/refine hooks 未移植；当前只接受调用方提供的一段反馈字符串。
- [builtin_modes.py](../graph-as-policy/gap/benchmark/builtin_modes.py) 中 `llm_generation` 为一次生成，不包含 rehearsal/refine 循环。

所以，“能修复生成的 Python 语法/接口错误”与“能根据机械臂实际失败自动学习改进策略”需要分别验证。调用方可以利用已有反馈字符串接口自行建设后一个循环，但应将新增实现与作者公开功能区分。

还检查了官方主仓库另两个公开分支，以及 `graph-robots/gap`、`graph-as-policy-anonymous`。后两者是项目展示网站，没有找到另一个公开的完整研究运行栈；对应目录树保存在本报告目录。这个结论限于本次核查的官方入口与固定版本，不推断作者未来的发布情况。

## 4. 本机能复现到哪里

本机为 WSL2、约 30 GB 内存、RTX 5060 Laptop（8151 MiB 显存）；当前 PATH 中没有 `nvcc`，当前进程未配置 OpenRouter/Gemini/Hugging Face 等相关环境变量。这里只记录是否存在变量，不读取或归档密钥值。[本机探测记录](source_environment_audit.json)

官方完整示例建议 RTX 4090 级、至少 24 GB 显存；安装文档称 LIBERO quickstart 单独约使用 10 GB。[官方安装说明](https://github.com/graph-robots/graph-as-policy/blob/93e929c9a1668cacba2e8be604e99a7538abbee9/docs/source/getting-started/installation.md)

因此，本机已经可以完成核心软件检查和图构建。要运行保留原模型的公开版完整抓放流程，还需补齐独立模型环境、CUDA 工具链、模型权重访问及模型 API，显存适配也需要实际验证。不能把 `.venv-audit` 的测试通过当作全栈安装成功。

即使换用符合官方建议的 GPU，论文缺失的研究代码仍需另行补齐。这是**发布内容的限制**，与硬件不足是两类不同问题。

## 5. 建议采用的复现路线

先以作者公开版的 **Franka + LIBERO/MuJoCo 抓放示例**为对象，保持机器人、技能与仿真器一致，减少适配变量。完成模型环境后，先执行作者提供的图，再真正从指令生成新图，最后跑官方小规模评测。每一步保留固定版本、输入、生成图、检查点、运行日志和视频。

接着再研究 self-learning：基于论文和已有 feedback/trace 接口，独立实现“执行 → 评价 → 分析失败 → 改图 → 再执行”的循环，与一次生成、人工图和仅调参数做对照。这个成果应明确称为**依据论文实现的自学习扩展**；若希望严格原样复现，则需要获得作者未发布的研究组件与实验配置。

本轮尚未发起任何真实模型调用，没有论文成功率或机械臂演示结果。后续公开版全流程需要确定可用 GPU、模型服务及调用预算；论文级复现还需要决定等待/取得原研究代码，还是采用可公开审查的独立实现。
