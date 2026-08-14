#!/bin/sh
# cam_probe.sh — USB 카메라를 꽂고 이걸 돌리면 정체가 드러난다.
# 사양을 몰라도 UVC 여부·해상도·포맷·컨트롤·실제 캡처까지 한 번에 확인한다.
#
#   sh cam_probe.sh              # 붙어 있는 전체
#   sh cam_probe.sh /dev/video0  # 특정 장치만
#
# 열화상은 대개 Y16(16bit raw) 또는 UYVY로 잡힌다. Y16이 보이면 온도 데이터가 그대로 나오는
# 것이고, 변환식(스케일·오프셋)은 제조사 스펙이 있어야 픽셀→℃로 바꿀 수 있다.

echo "=== USB 장치 ==="
lsusb | grep -viE "root hub|Linux Foundation"

echo
echo "=== UVC 인식 ==="
lsmod | grep -q "^uvcvideo" && echo "  uvcvideo 로드됨" || echo "  ⚠ uvcvideo 미로드 (sudo modprobe uvcvideo)"
dmesg 2>/dev/null | grep -iE "uvcvideo|Found UVC" | tail -5

echo
echo "=== USB 링크 속도 (3.0 = 5000M) ==="
lsusb -t | grep -vE "Class=root_hub"

DEVS="$*"
[ -z "$DEVS" ] && DEVS=$(ls /dev/video* 2>/dev/null | grep -vE "dec|enc")
[ -z "$DEVS" ] && { echo; echo "⚠ /dev/video* 없음 — 카메라가 안 잡혔다"; exit 1; }

for d in $DEVS; do
    echo
    echo "════════ $d ════════"
    v4l2-ctl -d "$d" --info 2>&1 | grep -E "Driver name|Card type|Bus info|Device Caps" -A0
    echo
    echo "── 포맷/해상도/프레임레이트 ──"
    v4l2-ctl -d "$d" --list-formats-ext 2>&1 | head -60
    echo
    echo "── 컨트롤 ──"
    v4l2-ctl -d "$d" --list-ctrls 2>&1 | head -30

    # 실제로 프레임이 나오는지. 기본 포맷 그대로 5장만.
    # ⚠ timeout 필수 — 스트림이 안 켜지는 장치(예: CDC로 초기화가 필요한 열화상)에서
    #   v4l2-ctl이 영원히 멈춰 /dev/video를 붙잡는다. 그러면 다음 실행이 EBUSY로 죽는다.
    echo
    echo "── 캡처 테스트 (5프레임, 8초 제한) ──"
    out="/tmp/$(basename "$d")_probe.raw"
    if timeout 8 v4l2-ctl -d "$d" --stream-mmap --stream-count=5 --stream-to="$out" 2>&1 | tail -3; then
        :
    fi
    if [ -s "$out" ]; then
        echo "  받음: $(stat -c%s "$out") bytes → $out"
    else
        echo "  ⚠ 프레임이 나오지 않는다 — 스트림 시작에 벤더 초기화가 필요할 수 있다"
        echo "    (CDC 시리얼 /dev/ttyACM* 이 같이 잡혔다면 그쪽으로 명령을 보내야 하는 장치다)"
    fi
done

echo
echo "─────────────────────────────────────────────"
echo "Y16이 보이면 열화상 raw. 픽셀→℃ 변환식은 제조사 스펙 필요."
echo "MJPEG/UYVY만 보이면 일반 UVC 카메라로 바로 쓸 수 있다."
