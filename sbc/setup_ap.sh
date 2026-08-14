#!/bin/bash
# setup_ap.sh — 무선을 AP(액세스포인트) 모드로 바꾼다.
#
#   sudo ./setup_ap.sh          AP 켜기
#   sudo ./setup_ap.sh off      원래대로 (공유기 클라이언트)
#   sudo ./setup_ap.sh status   현재 상태
#
# 로봇이 자체 네트워크를 만들고 IPC 가 거기 붙는 구성이다.
# 공유기가 없는 현장에서도 IPC ↔ 로봇이 바로 통신할 수 있다.
#
# ⚠ AP 로 바꾸면 기존 무선 접속(공유기 경유)은 끊긴다.
#   **유선이 살아 있는 상태에서 실행할 것.** 안 그러면 접속을 잃는다.
set -euo pipefail

WIFI_DEV=wlP4p65s0
CON_AP=navi-ap
CON_CLIENT="navifra"          # 기존 공유기 접속 프로파일

# AP 설정. IPC 는 이 대역에서 주소를 받는다.
AP_SSID=${NAVI_AP_SSID:-EV-DL_AP}
AP_PASS=${NAVI_AP_PASS:-123123123}         # 🔴 WPA2 는 8자 미만을 거부한다 (123123 은 못 쓴다)
AP_ADDR=${NAVI_AP_ADDR:-192.168.50.1/24}   # 로봇 자신의 주소
AP_BAND=${NAVI_AP_BAND:-bg}                # bg=2.4GHz(도달거리) / a=5GHz(속도)
AP_CHAN=${NAVI_AP_CHAN:-6}

usage() { sed -n '2,12p' "$0"; exit 1; }

show_status() {
    echo "=== 무선 상태 ==="
    nmcli -t -f NAME,DEVICE,STATE connection show --active 2>/dev/null | grep -i "$WIFI_DEV" || echo "  (무선 비활성)"
    ip -br addr show "$WIFI_DEV" 2>/dev/null || true
    echo "=== 유선 (이게 살아 있어야 안전하다) ==="
    ip -br addr show end1 2>/dev/null || true
    if nmcli -t -f NAME connection show --active 2>/dev/null | grep -q "^$CON_AP$"; then
        echo ""
        echo "AP 동작 중 — SSID: $(nmcli -g 802-11-wireless.ssid connection show $CON_AP 2>/dev/null)"
        echo "  접속 주소: ${AP_ADDR%/*}"
        echo "  붙은 단말:"
        iw dev "$WIFI_DEV" station dump 2>/dev/null | grep -c Station | sed 's/^/    /' || echo "    0"
    fi
}

case "${1:-on}" in
  status) show_status; exit 0 ;;
  off)
    echo "── AP 끄고 공유기 접속으로 되돌린다"
    # autoconnect 도 함께 되돌린다 — 안 그러면 재부팅 때 AP 가 다시 올라온다
    nmcli connection modify "$CON_AP" connection.autoconnect no 2>/dev/null || true
    nmcli connection modify "$CON_CLIENT" connection.autoconnect yes 2>/dev/null || true
    nmcli connection down "$CON_AP" 2>/dev/null || true
    nmcli connection up "$CON_CLIENT" 2>/dev/null || echo "  (기존 프로파일 '$CON_CLIENT' 없음 — 수동으로 붙일 것)"
    sleep 3; show_status; exit 0 ;;
  on) ;;
  *) usage ;;
esac

# 유선이 없는데 AP 로 바꾸면 접속을 잃는다. 막아준다.
if ! ip -br addr show end1 2>/dev/null | grep -q "UP"; then
    echo "🔴 유선(end1)이 안 올라와 있다. AP 로 바꾸면 접속을 잃는다."
    echo "   유선을 연결하고 다시 실행할 것. (정말 강행하려면 NAVI_AP_FORCE=1)"
    [ "${NAVI_AP_FORCE:-0}" = "1" ] || exit 1
fi

echo "── AP 설정: SSID=$AP_SSID  주소=$AP_ADDR  밴드=$AP_BAND ch$AP_CHAN"

# 기존 AP 프로파일이 있으면 지우고 새로 만든다 (설정이 누적돼 꼬이는 걸 막는다)
nmcli connection delete "$CON_AP" 2>/dev/null || true

nmcli connection add type wifi ifname "$WIFI_DEV" con-name "$CON_AP" \
    autoconnect yes ssid "$AP_SSID"

# 🔴 재부팅 시 어느 쪽이 뜰지 확실히 해둔다.
#    AP 와 공유기 프로파일이 둘 다 autoconnect=yes 이고 우선순위가 같으면(둘 다 0)
#    부팅 때마다 다른 쪽이 올라올 수 있다. AP 를 명시적으로 이기게 한다.
#    (공유기 프로파일은 지우지 않는다 — `setup_ap.sh off` 로 되돌릴 때 쓴다)
nmcli connection modify "$CON_AP" connection.autoconnect-priority 100
nmcli connection modify "$CON_CLIENT" connection.autoconnect no 2>/dev/null || true

nmcli connection modify "$CON_AP" \
    802-11-wireless.mode ap \
    802-11-wireless.band "$AP_BAND" \
    802-11-wireless.channel "$AP_CHAN" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$AP_PASS" \
    ipv4.method shared \
    ipv4.addresses "$AP_ADDR" \
    ipv6.method disabled

# ipv4.method=shared 면 NetworkManager 가 DHCP 서버까지 띄운다 —
# IPC 는 아무 설정 없이 꽂기만 하면 주소를 받는다.
# (고정 IP 로 쓰고 싶으면 IPC 쪽에서 이 대역의 주소를 직접 잡으면 된다)

echo "── 기존 공유기 접속 내리고 AP 올린다"
nmcli connection down "$CON_CLIENT" 2>/dev/null || true
nmcli connection up "$CON_AP"

sleep 4
echo ""
show_status
echo ""
echo "IPC 에서 SSID '$AP_SSID' 로 접속 (비밀번호: $AP_PASS)"
echo "로봇 주소: ${AP_ADDR%/*}   — MQTT ${AP_ADDR%/*}:1883"
