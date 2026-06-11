# VS Code 终端启动文件：先加载用户 bashrc，再激活 env_isaaclab
# 路径相对本文件解析（.vscode/ → 仓库根的上一级 → env_isaaclab），与终端 cwd 无关
[ -f ~/.bashrc ] && source ~/.bashrc
source "$(dirname "${BASH_SOURCE[0]}")/../../env_isaaclab/bin/activate"
