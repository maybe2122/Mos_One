# 🗂️ 版本记录系统说明

这个仓库用**三层文档 + git tag** 记录演进。三层各回答一个不同的问题，互不重复：

| 层 | 文件 | 回答 | 朝向 | 粒度 |
|---|---|---|---|---|
| **前瞻计划** | [`todo.md`](../todo.md) | 接下来做什么 | 向前看 | 阶段 / 待办 |
| **工程里程碑** | [`CHANGELOG.md`](../CHANGELOG.md) + git tag | 代码现在能做什么了 | 向后看 | 每个能力里程碑 |
| **实验迭代台账** | [`doc/experiments/EXPERIMENTS.md`](experiments/EXPERIMENTS.md) | 我试了什么、为什么、学到什么 | 向后看 | 每次训练/调参 |

核心原则：**台账不复制 SwanLab/TensorBoard 已有的指标曲线**，它记录工具抓不到的*叙事与决策*（为什么改、结论、下一步），用链接指回 SwanLab run / 训练目录 / git commit。指标曲线归 SwanLab，叙事归台账，里程碑归 CHANGELOG，计划归 todo。

## 闭环怎么转

```
todo.md「下一步」                       想做的事
   │  立项（分配 EXP-XXXX，状态 📋 计划）
   ▼
EXPERIMENTS.md                          一次训练/调参的因果链
   │  跑完原地回填结果 + 链接（python tools/exp/log_run.py）
   │  状态 📋→✅，SwanLab 曲线在此链接
   ▼
达成可演示能力 → 打 tag + 写 CHANGELOG    代码能力快照
```

- `todo.md` 里某条"下一步"开干时，去 `EXPERIMENTS.md` 用模板（或脚本）立一个 `EXP-XXXX`，状态 `📋 计划`。
- 训练跑完，用脚本把结果回填进**同一个** EXP（ID 不变，状态改 `✅/❌`），SwanLab run 链接也补在这里。
- 当若干实验累积成一个**可演示的整机/管线能力**（如"能 trot 了"），在对应 commit 打里程碑 tag，并在 `CHANGELOG.md` 写一节。

## 日常操作

**记一次实验**（训练跑完后，最常用）：
```bash
python3 tools/exp/log_run.py \
  --run-dir logs/rsl_rl/<exp>/<ts>_<run_name> \
  --title "指令条件化重训 (obs 45->48)" \
  --hypothesis "指令进 obs 后策略能响应 cmd_vx" \
  --changes "obs 45->48；同步 rl_deploy._build_obs + policy_export.OBS_DIM" \
  --metrics "reward 18.2->19.1, near_limit_frac 0.81->0.62" \
  --conclusion "速度响应成立；shank 饱和仍偏高 -> 下一步加力矩惩罚" \
  --swanlab-url <粘贴 SwanLab run 链接>
# 先 --dry-run 预览；--run-dir 在远端训练本地没有时可省略
```
脚本自动填：编号递增、git 短哈希(+dirty)、最近 tag、run 名、seed/iters（从 `params/agent.yaml`）。

**立一个计划项**：复制 [`experiments/TEMPLATE.md`](experiments/TEMPLATE.md) 到台账，状态写 `📋 计划`，结论留空。

**打一个里程碑**：见下。

## git tag 里程碑规范

- 命名：`vX.Y-<能力关键词>`，例 `v0.3-standup`、`v0.5-control-stack`。
- 时机：达成一个**可演示**的能力（不是每次提交），与 CHANGELOG 的版本节一一对应。
- 用**带注释 tag**（`-a`），信息一句话写清这个里程碑能干什么。

### 补打历史里程碑 tag

下面这组命令把已有 5 个里程碑 tag 补到对应历史 commit 上（已审阅 git log 选定）。
**先看一眼再执行**，确认无误后逐条运行：

```bash
git tag -a v0.1-sim-setup     3c964f5 -m "Isaac Lab 接入 + 闭链 USD 环境跑通"
git tag -a v0.2-motor-gui     bf44551 -m "电机调试 + 站立标定/方向验证网页工具"
git tag -a v0.3-standup       35e3ab2 -m "真机站起来：趴姿安全启动->站姿，前馈力矩/健壮性"
git tag -a v0.4-deploy        bc2cf1b -m "50Hz RL 推理部署 + SwanLab 实验跟踪接入"
git tag -a v0.5-control-stack f51ee93 -m "运控栈：FK/IK + trot + 域随机化 + ONNX 导出"
```

打完用 `git tag -l` 确认。若要推到远端：`git push origin --tags`（**推送会公开这些 tag，确认后再推**）。

下一个里程碑（当前"未发布"段的力矩分析等）成熟后：
```bash
git tag -a v0.6-<关键词> -m "一句话能力"   # 打在当时的 HEAD
```
然后把 `CHANGELOG.md` 的 `[未发布]` 内容迁入新版本节。
