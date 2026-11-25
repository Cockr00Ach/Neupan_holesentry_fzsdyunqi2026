#!/bin/bash
# 把整个工作区（不含 build/install/log、录制数据、先验点云）同步并推送到 GitHub
#
#   GITHUB_TOKEN=ghp_xxx ./push_github.sh ["commit message"]
#
# 首次使用会创建 ~/fzsd_project_snapshot 作为“可推送快照”仓库。
set +u
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SNAP=${SNAP:-$HOME/fzsd_project_snapshot}
REPO=${REPO:-https://github.com/Cockr00Ach/Neupan_holesentry_fzsdyunqi2026.git}
MSG=${1:-"sync: $(date '+%Y-%m-%d %H:%M')"}
: "${GITHUB_TOKEN:?请用 GITHUB_TOKEN=ghp_xxx ./push_github.sh}"

mkdir -p "$SNAP"
rsync -a --delete \
    --exclude='build/' --exclude='install/' --exclude='log/' --exclude='.cache/' \
    --exclude='*.tar.xz' --exclude='*.pcd' --exclude='__pycache__/' --exclude='*.pyc' \
    --exclude='recorddata/' --exclude='cmdtreeautoaim.zip' --exclude='.git/' \
    --exclude='.pytest_cache/' --exclude='*.olx' --exclude='*.bin' \
    --exclude='/.gitignore' --exclude='/README.md' \
    "$WS/" "$SNAP/"

cd "$SNAP" || exit 1
[ -d .git ] || git init -q
git config user.name "${GIT_NAME:-baymaxwish}"
git config user.email "${GIT_EMAIL:-baymaxwish@localhost}"
git add -A
if git diff --cached --quiet; then
    echo "没有变化，跳过提交"
else
    git commit -q -m "$MSG"
fi
[ "$(git branch --show-current)" = "main" ] || git branch -m "$(git branch --show-current)" main
git remote add fzsd "$REPO" 2>/dev/null || git remote set-url fzsd "$REPO"
AUTH=$(printf 'x-access-token:%s' "$GITHUB_TOKEN" | base64 -w0)
GIT_TERMINAL_PROMPT=0 git -c http.extraheader="AUTHORIZATION: basic $AUTH" push -f -u fzsd main
