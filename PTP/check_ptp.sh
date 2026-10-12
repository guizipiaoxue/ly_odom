#!/usr/bin/env bash
# 仅检查 PTP 状态或旁路验收，不提供每次启动/停止授时的入口。
set -euo pipefail
PTP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PTP_ACTION="${1:-status}"
if [[ "$PTP_ACTION" == -h || "$PTP_ACTION" == --help ]]; then
    echo '用法：./PTP/check_ptp.sh status|verify [有线网卡]'
    echo 'status 只读；verify 需要 sudo 和正在输出的雷达数据。PTP 由 systemd 开机自启。'
    exit 0
fi
PTP_INTERFACE="${2:-$(python3 "$PTP_DIR/host_check.py" --interface-only)}"
[[ "$PTP_INTERFACE" =~ ^[a-zA-Z0-9_.-]+$ ]] || { echo '网卡无效。' >&2; exit 1; }
if systemctl cat "sentry-ptp4l@$PTP_INTERFACE.service" >/dev/null 2>&1 &&
   [[ -f "/etc/linuxptp/sentry/master-$PTP_INTERFACE.conf" ]]; then
    PTP_PREFIX=sentry
    PTP_SOCKET="/run/ptp4l-$PTP_INTERFACE"
else
    PTP_PREFIX=ly
    PTP_SOCKET="/run/ly-ptp4l-$PTP_INTERFACE"
fi
PTP_MASTER="$PTP_PREFIX-ptp4l@$PTP_INTERFACE"
PTP_SERVO="$PTP_PREFIX-phc2sys@$PTP_INTERFACE"
case "$PTP_ACTION" in
    status)
        python3 "$PTP_DIR/host_check.py" --interface "$PTP_INTERFACE" || true
        systemctl is-enabled "$PTP_MASTER" "$PTP_SERVO" || true
        systemctl status "$PTP_MASTER" "$PTP_SERVO" --no-pager || true
        journalctl -u "$PTP_SERVO" -n 8 --no-pager
        if (( EUID == 0 )) && [[ -S "$PTP_SOCKET" ]]; then
            pmc -u -b 0 -s "$PTP_SOCKET" 'GET PORT_DATA_SET' 'GET TIME_PROPERTIES_DATA_SET'
        fi
        ;;
    verify)
        (( EUID == 0 )) || { echo "请执行 sudo ./PTP/check_ptp.sh verify $PTP_INTERFACE" >&2; exit 1; }
        # 每个单元单独检查，要求两个服务都运行。
        systemctl is-active --quiet "$PTP_MASTER"
        systemctl is-active --quiet "$PTP_SERVO"
        exec python3 "$PTP_DIR/monitor_packets.py" --interface "$PTP_INTERFACE" --seconds 10
        ;;
    *) echo '只支持 status/verify；服务由 systemd 开机自启。' >&2; exit 2 ;;
esac
