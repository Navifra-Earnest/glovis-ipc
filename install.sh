#!/bin/bash
# IPC 세팅 — 이 저장소를 IPC 의 ~/glovis 로 클론한 뒤 여기서 실행한다.
#
#   ssh navifra@<IPC>
#   git clone <이 저장소> ~/glovis && ~/glovis/install.sh
#
# 갱신은 `git pull && ~/glovis/install.sh` 또는 코드만 바뀐 경우
# `systemctl --user restart joy-teleop crevis-io navi-console`.
#
# 멱등하다 — 몇 번 돌려도 같은 결과다.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
HERE=$(pwd)

# 서비스 유닛이 %h/glovis/*.py 를 가리킨다 → 경로가 고정이어야 한다.
if [ "$HERE" != "$HOME/glovis" ]; then
    echo "❌ 이 저장소는 ~/glovis 에 있어야 한다 (유닛의 ExecStart 가 %h/glovis 고정)."
    echo "   현재: $HERE"
    exit 1
fi

echo "── 의존성 확인"
miss=()
python3 -c "import paho.mqtt.client" 2>/dev/null || miss+=(python3-paho-mqtt)
command -v mosquitto_pub >/dev/null                || miss+=(mosquitto-clients)
command -v ffmpeg        >/dev/null                || miss+=(ffmpeg)
if [ ${#miss[@]} -gt 0 ]; then
    echo "   ⚠️ 빠진 패키지: ${miss[*]}"
    echo "      sudo apt install -y ${miss[*]}"
    echo "   설치 후 다시 실행할 것."
    exit 1
fi
echo "   OK (paho-mqtt · mosquitto-clients · ffmpeg)"

echo "── 서비스 유닛 설치"
mkdir -p "$HOME/.config/systemd/user"
install -m 644 ./*.service "$HOME/.config/systemd/user/"
systemctl --user daemon-reload

echo "── 서비스 기동"
# navi-console 은 graphical-session 에 물려 있다(창을 띄우므로).
# 나머지 둘은 default.target — 자동로그인으로 세션이 생기면 같이 뜬다.
systemctl --user enable --now navi-console joy-teleop crevis-io

echo
echo "── 상태"
for s in navi-console joy-teleop crevis-io; do
    printf "   %-14s %s\n" "$s" "$(systemctl --user is-active "$s")"
done
echo
echo "다음:"
echo "  journalctl --user -u joy-teleop -f      # 주행 로그"
echo "  journalctl --user -u crevis-io -f       # 물리버튼 로그"
echo "  python3 joy_teleop.py --selftest        # 기구학·인터락 검증(HW 불필요)"
echo "  python3 crevis_io.py --selftest         # 버튼 판정 검증"
echo
echo "⚠️ 리셋 버튼(navi 재시작)은 별도 준비가 필요하다 — README 의 '리셋 버튼 전제' 참고."
