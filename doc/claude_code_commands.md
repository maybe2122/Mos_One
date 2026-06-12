# Claude Code 常用命令速查

> 整理日期:2026-06-12。分四部分:内置斜杠命令、本仓库会话可用的技能命令、输入快捷方式、终端 CLI 启动参数。
> 注:没有内置的 `/goal` 命令;管理目标/任务一般用计划模式(Shift+Tab)或 `/todos`。

## 一、内置斜杠命令(在对话框输入)

### 日常高频

| 命令 | 作用 |
|---|---|
| `/help` | 查看帮助和可用命令列表 |
| `/clear` | 清空当前对话上下文,开新会话(不退出) |
| `/compact` | 压缩对话历史,腾出上下文空间(可加提示词指定保留重点) |
| `/context` | 查看当前上下文占用情况 |
| `/resume` | 恢复之前的某个会话 |
| `/rewind` | 回退到对话中的某个早期检查点(可同时回退代码改动) |
| `/export` | 导出当前对话内容 |
| `/todos` | 查看当前任务清单 |
| `/cost` / `/usage` | 查看本会话花费 / 用量额度 |

### 配置与模型

| 命令 | 作用 |
|---|---|
| `/config` | 打开设置面板(主题、模型等简单设置) |
| `/model` | 切换模型(Opus / Sonnet / Haiku 等) |
| `/fast` | 切换快速模式(Opus 提速输出,不降级小模型) |
| `/permissions` | 管理工具权限白名单 |
| `/memory` | 编辑记忆文件(CLAUDE.md 等) |
| `/output-style` | 切换输出风格 |
| `/statusline` | 配置底部状态栏 |
| `/vim` | 启用 Vim 键位编辑输入 |
| `/terminal-setup` | 配置终端按键(如 Shift+Enter 换行) |

### 工程协作

| 命令 | 作用 |
|---|---|
| `/init` | 扫描代码库,生成 CLAUDE.md 项目说明 |
| `/review` | 审阅一个 GitHub Pull Request |
| `/pr-comments` | 拉取并查看 PR 上的评论 |
| `/install-github-app` | 安装 GitHub 集成(@claude 评论触发) |
| `/agents` | 管理子 agent 定义 |
| `/hooks` | 管理钩子(工具调用前后自动执行的脚本) |
| `/mcp` | 管理 MCP 服务器连接 |
| `/bashes` | 查看/管理后台运行的 shell 任务 |

### 诊断与账号

| 命令 | 作用 |
|---|---|
| `/status` | 查看会话状态(模型、目录、账号等) |
| `/doctor` | 自检安装/配置问题 |
| `/bug` | 向 Anthropic 反馈问题 |
| `/login` / `/logout` | 登录 / 登出 |
| `/add-dir` | 把额外目录加入工作区 |

## 二、本会话可用的技能命令(Skills)

这些是当前环境里注册的技能,比内置命令更"重",通常会启动一套完整流程:

| 命令 | 作用 |
|---|---|
| `/code-review` | 审查当前分支 diff,找正确性 bug 和可简化处。力度可选 `low`/`medium`(只报高置信度)到 `high`/`max`(覆盖更广);`--fix` 直接修到工作区,`--comment` 发 PR 行内评论 |
| `/code-review ultra` | 多 agent 云端深度审查当前分支(或 `ultra <PR号>` 审 GitHub PR)。计费,需手动触发 |
| `/security-review` | 对当前分支待提交改动做安全审查 |
| `/simplify` | 只做简化/复用/效率清理并直接应用(不找 bug) |
| `/verify` | 实际运行应用验证某个改动是否生效(不只是跑测试) |
| `/run` | 启动本项目的应用看效果(支持截图) |
| `/loop` | 按间隔循环执行某个提示或命令,如 `/loop 5m /foo`;不带间隔则自动调节节奏 |
| `/schedule` | 创建/管理定时云端任务(cron 式,也支持一次性定时) |
| `/fewer-permission-prompts` | 扫描历史,把常用只读命令加入白名单减少权限弹窗 |
| `/keybindings-help` | 自定义键盘快捷键(~/.claude/keybindings.json) |
| `/claude-api` | Claude API / Anthropic SDK 速查(模型 id、价格、流式、工具调用等) |

## 三、输入快捷方式

| 输入 | 作用 |
|---|---|
| `! <命令>` | 直接在会话里执行 shell 命令,输出进入对话(适合交互式登录如 `gcloud auth login`) |
| `@文件路径` | 引用文件,内容带入上下文(支持 Tab 补全) |
| `# <内容>` | 快速把一条笔记追加进记忆(CLAUDE.md) |
| `Shift+Tab` | 循环切换权限模式:普通 → 自动接受编辑 → 计划模式(plan mode,只规划不动手) |
| `Esc` | 打断 Claude 当前操作 |
| `Esc Esc` | 跳回历史消息(配合 /rewind 回退) |
| `Ctrl+R` | 展开查看完整输出(verbose) |
| `Ctrl+B` | 把当前任务转后台运行 |

## 四、终端 CLI 启动参数(`claude ...`)

| 命令 | 作用 |
|---|---|
| `claude` | 启动交互会话 |
| `claude "提示词"` | 带初始提示启动 |
| `claude -p "提示词"` | 无头模式:执行完直接打印结果退出(适合脚本/管道) |
| `claude -c` | 继续最近一次会话 |
| `claude -r <会话id>` | 恢复指定会话 |
| `claude --model <模型>` | 指定模型启动 |
| `claude commit` | 让 Claude 创建一次 git 提交 |
| `claude mcp` | 配置 MCP 服务器 |
| `claude update` | 更新 Claude Code |

## 五、本项目典型用法举例

```bash
# 改完 him/train.py 之后审一遍
/code-review

# 训练脚本改动想确认真的能跑
/verify him/train.py 的 --payload 参数在冒烟训练中生效

# 每 10 分钟看一眼后台训练日志
/loop 10m 检查 /tmp/payload_smoke.log 末尾有没有报错
```
