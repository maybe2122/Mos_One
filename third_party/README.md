# 📦 third_party/ — 外部依赖（git submodule）

> 本目录只放**外部上游项目**的 submodule（仓库里仅存 commit 指针，不存代码）。
> 自研代码一律不放这里：训练在 `source/`+`scripts/`，部署在 `deploy/`，电机工具在 `motor_control/`。

## 当前内容

| 子模块 | 上游 | 用途 |
|:---|:---|:---|
| `rl_sar/` | [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) | RL 真机部署框架（C++/ROS）。`deploy/real/policy_export.py` 导出的 TorchScript 模型与 `deploy/real/config/mos2026_2.yaml` 均按其 schema 设计——模型+配置拷到 `<rl_sar>/policy/mos2026_2/` 即可用它的 C++ 控制栈上真机。与自研 Python 部署路线（`deploy/real/rl_deploy.py`）并行互补。 |
| `mjlab/` | [mujocolab/mjlab](https://github.com/mujocolab/mjlab) | MuJoCo Warp 上的 Isaac Lab 式训练框架（manager-based + rsl_rl）。用于轻量本地训练与 sim2sim 验证（闭链 equality 原生支持）。**独立 uv 环境**：`cd third_party/mjlab && uv sync --extra cu128`，入口 `uv run train/play/list-envs`。集成说明见 [`doc/mjlab_integration.md`](../doc/mjlab_integration.md)。 |

## 常用操作

```bash
# 新 clone 后拉取子模块（不拉不影响训练/仿真，只影响 rl_sar 部署路线）
git submodule update --init --depth 1

# 跟进上游更新（更新后记得 commit 新的 gitlink 指针）
git submodule update --remote third_party/rl_sar
```

> 📌 mjlab 已按 [`doc/mjlab_integration.md`](../doc/mjlab_integration.md) 的 A 方案集成（2026-06-11，
> 官方 Go1 任务冒烟通过）；其 `.venv`/`logs` 等运行产物不要提交——在 submodule 里跑完训练记得清理。
