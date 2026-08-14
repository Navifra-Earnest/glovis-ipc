#!/bin/sh
# exp3b.sh — VPU(mpph264enc) + RTSP 전송 조합 재현 시험.
# 크래시 났던 조합이다. autosuspend 수정 후에도 재현되는지 본다.
#
# ⚠ videoconvert 뒤 format=NV12 를 빼지 말 것. 빼면 UYVY 가 그대로 인코더로 가서
#   RGA 드라이버가 커널 메모리 회계를 깨뜨리고 보드가 먹통이 된다. cam_stream.sh 주석 참고.
#
# ⚠ pkill -f 로 gst-launch 를 죽이지 말 것 — 그 명령을 담은 셸/SSH 세션의
#   커맨드라인에도 문자열이 걸려 자기 자신을 죽인다. killall 을 쓴다.
set -e
killall -q gst-launch-1.0 mediamtx 2>/dev/null || true
sleep 2

sudo -n sh -c 'echo "===== EXP3b VPU+RTSP start" > /dev/kmsg' 2>/dev/null || \
  echo radxa | sudo -S sh -c 'echo "===== EXP3b VPU+RTSP start" > /dev/kmsg' 2>/dev/null

nohup mediamtx /tmp/mtx_only.yml > /tmp/mtx_e.log 2>&1 &
sleep 5
echo "MediaMTX 리스너: $(grep -c listener /tmp/mtx_e.log)"

nohup gst-launch-1.0 \
    v4l2src device=/dev/video0 \
    ! video/x-raw,format=UYVY,width=1280,height=720 \
    ! videoconvert ! video/x-raw,format=NV12 \
    ! mpph264enc bps=2000000 rc-mode=cbr gop=30 max-pending=1 \
    ! h264parse config-interval=1 \
    ! rtspclientsink location=rtsp://127.0.0.1:8554/cam latency=0 protocols=tcp \
    > /tmp/e3b.log 2>&1 &

sleep 15
echo "gst 상태: $(grep -cE 'PLAYING|Pipeline is live' /tmp/e3b.log) (1 이상이면 재생 중)"
tail -2 /tmp/e3b.log
echo "MediaMTX publish: $(grep -ci 'is publishing' /tmp/mtx_e.log)"
