#!/bin/bash
# ir_stream.sh — IR 카메라를 H.264 로 인코딩해 내보낸다. (보드에서 실행)
#
#   sudo ./ir_stream.sh start          TCP 5000 으로 송출
#   sudo ./ir_stream.sh start rtsp     MediaMTX 로 밀어넣는다 (rtsp://보드:8554/ir)
#   sudo ./ir_stream.sh stop
#   sudo ./ir_stream.sh status
#
# 받는 쪽 (개발 PC):
#   ffplay -fflags nobuffer -flags low_delay tcp://192.168.0.64:5000
#   ffplay rtsp://192.168.0.64:8554/ir
#
# ⚠ navi 와 카메라를 함께 쓸 수 없다. UVC 는 한 프로세스만 연다 —
#   스트리밍을 켜려면 navi 를 멈춰야 하고, 그동안 IR 프레임 발행이 멈춘다.
#   (열화상·ToF·구동은 영향 없다)
#
# RK3588S 의 VPU 가 인코딩을 처리하므로 CPU 부하는 미미하다. UYVY 를 인코더가
# 직접 받으므로 videoconvert(CPU) 가 끼지 않는다 — 이게 성능의 핵심이다.
set -euo pipefail

DEV=${IR_DEV:-/dev/video0}
W=${IR_W:-1920}
H=${IR_H:-1080}
FPS=${IR_FPS:-30}
BPS=${IR_BPS:-2000000}
PORT=${IR_PORT:-5000}
RTSP=${IR_RTSP:-rtsp://127.0.0.1:8554/ir}
UNIT=ir-stream

enc="mpph264enc bps=$BPS rc-mode=cbr gop=$FPS"
src="v4l2src device=$DEV ! video/x-raw,format=UYVY,width=$W,height=$H,framerate=$FPS/1"
# config-interval=1 — SPS/PPS 를 매 IDR 마다 넣는다. 중간에 붙는 클라이언트가 바로 그린다.
parse="h264parse config-interval=1"

case "${1:-status}" in
  start)
    if systemctl is-active --quiet navi; then
        echo "⚠ navi 가 카메라를 점유 중이다. 먼저 멈춘다: sudo systemctl stop navi" >&2
        exit 1
    fi
    systemctl stop "$UNIT" 2>/dev/null || true
    if [ "${2:-tcp}" = "rtsp" ]; then
        command -v mediamtx >/dev/null || { echo "🔴 mediamtx 가 없다" >&2; exit 1; }
        systemctl is-active --quiet mediamtx || echo "[!] mediamtx 가 안 떠 있다 — 먼저 띄울 것"
        sink="rtspclientsink location=$RTSP"
        gst-inspect-1.0 rtspclientsink >/dev/null 2>&1 \
            || { echo "🔴 rtspclientsink 가 없다 (gstreamer1.0-rtsp 필요). tcp 모드를 쓸 것" >&2; exit 1; }
    else
        sink="video/x-h264,stream-format=byte-stream ! tcpserversink host=0.0.0.0 port=$PORT"
    fi
    # ssh 로 띄우면 세션이 끊길 때 같이 죽는다 — systemd 단위로 올린다
    systemd-run --unit="$UNIT" --collect --property=Restart=no \
        gst-launch-1.0 -q $src ! $enc ! $parse ! $sink >/dev/null
    sleep 3
    if systemctl is-active --quiet "$UNIT"; then
        echo "송출 시작 — ${2:-tcp}"
        [ "${2:-tcp}" = "rtsp" ] && echo "  ffplay $RTSP" \
                                 || echo "  ffplay -fflags nobuffer tcp://\$(hostname -I | awk '{print \$1}'):$PORT"
    else
        echo "🔴 시작 실패:"; journalctl -u "$UNIT" -n 8 --no-pager | tail -6
        exit 1
    fi
    ;;
  stop)
    systemctl stop "$UNIT" 2>/dev/null || true
    echo "송출 중지"
    ;;
  status)
    systemctl is-active "$UNIT" 2>/dev/null || echo inactive
    ss -tlnp 2>/dev/null | grep ":$PORT" || true
    ;;
  *)
    sed -n '2,14p' "$0"; exit 1 ;;
esac
