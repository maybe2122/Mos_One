<!--
单条实验台账模板。新建实验时：
  1) 复制下面 "## EXP-XXXX" 整段到 EXPERIMENTS.md 的 <!-- ENTRIES:START --> 之后（最新在上）；
  2) 在索引表 <!-- INDEX:ROWS --> 之后加一行；
  3) 把 XXXX 换成下一个递增编号（看现有最大值 +1）。
推荐改用脚本自动完成：python tools/exp/log_run.py --help
字段填写原则：指标只放"工具抓不到的关键数字 + 指回 SwanLab 的链接"，不要把曲线复制进来。
-->

## EXP-XXXX
### <一句话标题> — <✅ 完成 | 🔄 进行中 | 📋 计划 | ❌ 放弃> (YYYY-MM-DD)

- **假设**: 我预期改 X 会带来 Y（可证伪地写清楚）。
- **改动**: 相对上一版具体改了什么（reward 权重 / obs 维度 / 超参 / DR 开关 / 网络结构…）。
  - sim2real 同步项（若 obs/action 维度变化，务必同步 `deploy/real/rl_deploy.py` 与 `policy_export.py`）。
- **环境**: env_isaaclab + RTX 5090 ｜ num_envs / iterations / seed。
- **commit/tag**: `<短哈希>` ｜ 关联 `<最近里程碑 tag>`（工作区脏则标 `+dirty`）。
- **训练目录**: `logs/rsl_rl/<experiment>/<timestamp>_<run_name>/`（含 `params/agent.yaml`、`model_final.pt`）。
- **指标**: SwanLab project=`mos_one-mos` exp=`<run basename>` <链接> ｜ 关键数字：reward A→B、`near_limit_frac` A→B、CoT…
- **结论**: 假设成立/证伪？学到什么？→ 下一步指向哪个 EXP 或 `todo.md` 条目。
- **真机**: 是否上机 / 表现 / 未上机。
