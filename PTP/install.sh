#!/usr/bin/env bash
# 一次性安装依赖并启用 PTP 开机自启；已有本机专用服务时优先复用。
set -euo pipefail
PTP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
    echo '用法：sudo ./PTP/install.sh [有线网卡]；一次性部署并启用 PTP 开机自启。'
    exit 0
fi
if (( EUID != 0 )); then
    echo '需要管理员权限安装软件包和 systemd 配置，请执行 sudo ./PTP/install.sh [网卡]。' >&2
    exit 1
fi
PTP_INTERFACE="${1:-}"
if [[ -z "$PTP_INTERFACE" ]]; then
    # 保留原默认目标的路由选卡逻辑，不依赖独立检查工具。
    PTP_INTERFACE="$(python3 - <<'PY'
import json
import subprocess
routes = json.loads(subprocess.check_output(
    ['ip', '-j', 'route', 'get', '192.168.1.3'], text=True, timeout=5))
if not routes or 'dev' not in routes[0]:
    raise SystemExit('未找到到 192.168.1.3 的路由，请显式指定有线网卡')
print(routes[0]['dev'])
PY
)"
fi
[[ "$PTP_INTERFACE" =~ ^[a-zA-Z0-9_.-]+$ && -d "/sys/class/net/$PTP_INTERFACE" ]] || {
    echo '网卡无效。' >&2; exit 1;
}
apt-get update
apt-get install -y linuxptp ethtool python3
ethtool -T "$PTP_INTERFACE"
python3 - "$PTP_INTERFACE" <<'PY'
import subprocess, sys
from pathlib import Path
interface = sys.argv[1]
clocks = list((Path('/sys/class/net') / interface / 'device/ptp').glob('ptp[0-9]*'))
capabilities = subprocess.check_output(['ethtool', '-T', interface], text=True)
if len(clocks) != 1 or not all(value in capabilities for value in
                             ('hardware-transmit', 'hardware-receive', 'hardware-raw-clock')):
    raise SystemExit('网卡不支持本硬件 PTP 方案；请选择支持收发硬件时间戳的有线网卡。')
PY
# 当前机器已有 sentry 实例；绝不让两个伺服器同时调节同一 PHC。
if systemctl cat "sentry-ptp4l@$PTP_INTERFACE.service" >/dev/null 2>&1 &&
   [[ -f "/etc/linuxptp/sentry/master-$PTP_INTERFACE.conf" ]]; then
    if ! systemctl is-active --quiet "sentry-ptp4l@$PTP_INTERFACE.service" &&
       { pgrep -x ptp4l >/dev/null || pgrep -x phc2sys >/dev/null; }; then
        echo '另有 PTP 进程，拒绝启动竞争实例。' >&2; exit 1
    fi
    echo "复用 sentry PTP 服务并确保开机自启，不覆盖配置或重启运行实例：$PTP_INTERFACE"
    systemctl enable --now "sentry-ptp4l@$PTP_INTERFACE.service" "sentry-phc2sys@$PTP_INTERFACE.service"
    systemctl status "sentry-ptp4l@$PTP_INTERFACE" "sentry-phc2sys@$PTP_INTERFACE" --no-pager
    echo '后续只需 ./scripts/odom.sh；部署与维护经验见 PTP/AGENTS.md。'
    exit 0
fi
if systemctl is-active --quiet "ly-ptp4l@$PTP_INTERFACE.service"; then
    echo "本项目 PTP 实例已在运行，确保开机自启并保留配置：$PTP_INTERFACE"
    systemctl enable --now "ly-ptp4l@$PTP_INTERFACE.service" "ly-phc2sys@$PTP_INTERFACE.service"
    systemctl status "ly-ptp4l@$PTP_INTERFACE" "ly-phc2sys@$PTP_INTERFACE" --no-pager
    exit 0
fi
# 未知 PTP 部署不自动接管；先核对所有实例的时钟方向与网卡。
if pgrep -x ptp4l >/dev/null || pgrep -x phc2sys >/dev/null; then
    echo '已有其他 PTP 进程；为避免多重调节，未安装第二套服务。请先核对现有部署。' >&2
    exit 1
fi
install -d /etc/linuxptp/ly-odom /usr/local/libexec/ly-ptp
sed "s/@INTERFACE@/$PTP_INTERFACE/g" "$PTP_DIR/master.conf.in" > "/etc/linuxptp/ly-odom/master-$PTP_INTERFACE.conf"
install -m 644 "$PTP_DIR/bootstrap.py" /usr/local/libexec/ly-ptp/
install -m 644 "$PTP_DIR"/systemd/*.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now "ly-ptp4l@$PTP_INTERFACE.service" "ly-phc2sys@$PTP_INTERFACE.service"
echo '部署完成，PTP 将在开机时自动启动。后续运行 ./scripts/odom.sh 即可启动雷达与里程计。'
