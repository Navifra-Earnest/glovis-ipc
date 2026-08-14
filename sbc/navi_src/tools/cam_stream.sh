#!/bin/sh
# cam_stream.sh — IR 카메라를 H.264로 인코딩해 MediaMTX에 밀어 넣는다.
# MediaMTX가 이걸 runOnInit로 실행하고, 브라우저는 WebRTC로 받아 본다.
#
#   sh cam_stream.sh [카드이름] [W] [H] [FPS] [비트레이트bps] [RTSP경로]
#
# 저지연을 위해 잡은 것:
#   · NV12로 변환 후 인코딩 — 🔴 UYVY를 mpph264enc에 직접 넣으면 커널이 깨진다.
#                        gst-inspect 상 mpph264enc는 UYVY를 받는다고 광고하지만,
#                        실제로는 RGA(하드웨어 2D 가속기)로 변환을 시도하고 RGA가
#                        UYVY를 모른다. 상세는 아래 "RGA 커널 버그" 주석 참고.
#                        NV12는 VPU 네이티브라 RGA를 안 거친다.
#   · gop = fps         — 키프레임 1초 간격. 짧을수록 화면 깨짐 복구가 빠르지만 대역폭이 는다
#   · max-pending=1     — 인코더가 프레임을 쌓아두지 않게. 쌓이는 만큼 그대로 지연이다
#   · profile=main      — **브라우저 WebRTC 재생에 필요** (2026-07-31 실측으로 확정)
#                         high(기본) : 인코딩·전송은 되는데 브라우저가 화면을 못 그린다
#                         main       : ✅ 브라우저에서 정상 재생
#                         baseline   : 🔴 절대 금지 — "10000 is unsupport format now" 를
#                                      무한 출력하며 segfault, 그 로그가 디스크·CPU를 다 먹어
#                                      보드가 응답 불가가 된다. level=40 도 같은 증상.
#   · rc-mode=cbr       — 무선에서 비트레이트가 튀지 않게
#   · latency=0         — rtspclientsink가 버퍼를 쌓지 않게. 버퍼 = 그대로 지연이다
#   · protocols=tcp     — 로컬 전송이라 손실이 없고, UDP 포트 협상 문제를 피한다
# set -e 는 쓰지 않는다 — gstreamer가 실패했을 때 아래 진단·정리 코드까지 건너뛴다.

# 🔴 폭주 방어 (2026-07-31)
#    mpph264enc 가 잘못된 설정을 만나면 같은 에러를 초당 수만 줄 쏟아낸다.
#    그걸 파일로 받으면 /tmp(tmpfs)가 RAM을 다 먹어 유저스페이스가 마비된다 —
#    커널은 살아있어서 화면도 나오고 ping 도 되는데 SSH만 안 되는 상태가 된다.
#    (profile=baseline 에서 실제로 겪었고, 그 뒤로도 같은 증상이 반복됐다)
#
#    출력을 파일로 쌓지 않는다 — 앞부분만 남기고 나머지는 계속 버린다.
#
#    ⚠ ulimit -f / -v 는 쓰지 말 것. 시도했다가 gstreamer 가 레지스트리 캐시
#      (~/.cache/gstreamer-1.0/, 수 MB)를 쓰면서 "File size limit exceeded" 로 죽었다.
#      이 셸의 자식 전부에 걸리므로 의도치 않은 곳을 때린다.
LOG=${NAVI_STREAM_LOG:-/tmp/cam_stream.log}
LOG_MAX_BYTES=${NAVI_STREAM_LOG_MAX:-262144}    # 256KB — 정상 동작에선 수십 바이트다

CARD=${1:-See3CAM}
W=${2:-1280}
H=${3:-720}
FPS=${4:-30}
BPS=${5:-2000000}
PATHNAME=${6:-cam}

# 인코더는 SoC마다 다르다 — 여기 말고 다른 데는 손대지 않게 밖으로 뺀다.
#   ROCK 3A  = RK3568  : VPU 있음. 단 RGA2만 있어 포맷 제약이 심하다(아래 주석)
#   ROCK 5A  = RK3588S : VPU 세대가 다르다. MPP 로그가 둘을 다르게 취급한다 —
#                        "Only rk3588's h264/265/jpeg ... can use frame parallel",
#                        "unable to create enc vp8 for soc rk3568 unsupported"
#                        RGA도 RGA3+RGA2라 UYVY 제약이 없을 수 있다.
#   NAVI_ENC=hw  하드웨어 VPU (기본)
#   NAVI_ENC=sw  소프트웨어 x264enc — VPU가 말썽이거나 없는 보드에서
case "${NAVI_ENC:-hw}" in
    sw) FMT=I420
        ENC="x264enc bitrate=$((BPS / 1000)) speed-preset=ultrafast tune=zerolatency key-int-max=$FPS" ;;
    *)  FMT=NV12
        ENC="mpph264enc bps=$BPS rc-mode=cbr gop=$FPS max-pending=1 profile=main" ;;
esac

# /dev/videoN 은 USB 꽂는 순서로 바뀐다. 카드 이름으로 찾는다.
DEV=""
for d in /dev/video*; do
    case "$d" in *dec*|*enc*) continue;; esac
    if v4l2-ctl -d "$d" --info 2>/dev/null | grep -q "$CARD"; then
        # 캡처 가능한 노드만 (UVC는 메타데이터 노드도 만든다)
        if v4l2-ctl -d "$d" --list-formats 2>/dev/null | grep -q "'UYVY'"; then
            DEV="$d"
            break
        fi
    fi
done
[ -z "$DEV" ] && { echo "카메라를 못 찾았다: $CARD" >&2; exit 1; }
echo "cam_stream: $CARD → $DEV  ${W}x${H}@${FPS} ${BPS}bps → rtsp://127.0.0.1:8554/$PATHNAME" >&2

# 🔴 framerate 를 caps 에 못 박지 말 것.
#    이 카메라의 UYVY 는 해상도마다 지원 fps 가 제각각이다 (실측):
#        1920x1080 → 60, 30
#        1280x720  → 80, 50      ← 30 없음
#         640x480  → 120         ← 15 없음
#    지원하지 않는 값을 넣으면 v4l2src 가 not-negotiated 로 죽는다.
#    그래서 카메라는 자기가 낼 수 있는 속도로 두고, videorate 로 원하는 fps 를 만든다.
#    (videorate 는 프레임을 버리기만 하므로 부하가 거의 없다)
# 🔴 RGA 커널 버그 — NV12 를 반드시 명시할 것 (2026-07-31, 보드 먹통의 원인)
#    mpph264enc 는 caps 상 UYVY 를 받는다고 광고한다. 그래서 videoconvert 뒤에
#    포맷을 안 박으면 gstreamer 가 "인코더가 UYVY 받네" 하고 변환 없이 통과시킨다.
#    그러면 mpp 가 UYVY→NV12 변환을 RGA(하드웨어 2D 가속기)에 맡기는데,
#    RGA 드라이버가 UYVY 를 인식하지 못하고 실패 경로에서 커널 메모리 회계를 깨뜨린다:
#        rga: Unsuport format [0xffffff]
#        rga: memory param: w = 1280, h = 720, f = UNF(0xffffff), size = 0
#        rga: Can not alloc rga_virt_addr ... submit failed!
#        BUG: Bad rss-counter state mm:... type:MM_FILEPAGES val:3590
#    커널은 안 죽으므로 화면(HDMI)은 계속 나오는데 유저스페이스만 마비된다 —
#    SSH 도 안 되고 네트워크만 죽은 것처럼 보이는 그 증상이 이것이다.
#    NV12 는 VPU 네이티브 포맷이라 RGA 를 거치지 않는다.
#    변환은 CPU(videoconvert)가 하지만 720p30 이면 코어 하나로 충분하다.
#
#    확인법: 스트리밍 중  journalctl -kf | grep -E "rga: |rss-counter"  가 조용해야 한다.
#    (mpp 로그의 stride 로도 알 수 있다. UYVY 면 stride=width×2, NV12 면 stride=width)
#
# gstreamer 출력을 head 로 통과시켜 상한을 건다. 상한에 닿으면 head 가 파이프를 닫고
# gst-launch 가 SIGPIPE 로 죽는다 — 폭주가 시스템을 먹기 전에 스스로 멈추는 셈이다.
# 정상 동작 중에는 -q 라서 출력이 거의 없으므로 이 상한에 닿지 않는다.
# 🔴 queue 를 빼지 말 것 (2026-07-31)
#    gstreamer 는 queue 가 없으면 소스부터 싱크까지 한 스레드에서 돈다.
#    그래서 어느 한 곳이 잠깐 막히면 파이프라인 전체가 멈추는데,
#    프로세스는 살아 있고 에러도 안 나서 "돌고 있는 것처럼" 보인다.
#    실제로 겪은 모습: CPU 0.0% / 누적 39초(12분간) / RTSP 수신 0바이트인데
#    mediamtx 는 "is publishing" 이라고 표시. 로그만 보면 정상이라 한참 헤맸다.
#
#    leaky=downstream — 밀리면 오래된 프레임을 버린다. 실시간 영상은 쌓인 프레임에
#    가치가 없고, 안 버리면 그게 그대로 지연이 되거나 위처럼 멈춘다.
#
#    IN_FPS: 카메라에서 받는 속도. UYVY 1280x720 은 80/50 만 되는데 기본이 80이라
#    30fps 를 쓰려고 8장 중 5장을 버리며 USB 대역폭을 최대로 쓰고 있었다. 50 이면 충분하다.
IN_FPS=${NAVI_IN_FPS:-50}

# 입력 경로 — USB 대역폭과 CPU 부하를 맞바꾼다.
#   uyvy  (기본) 무압축. 화질 손실이 없고 CPU 도 안 쓴다.
#               대신 1280x720@50 이 737Mbps 라 USB3 대역폭을 거의 다 쓴다.
#   mjpeg       카메라가 JPEG 로 압축해서 보낸다. USB 부하가 수십분의 1 로 준다.
#               `uvcvideo -71`(EPROTO)이 대역폭/신호 문제로 나는 거라면 이쪽이 답이다.
#               디코딩은 CPU 가 한다 — mppjpegdec(하드웨어)는 이 카메라 JPEG 를 못 읽는다
#               (`Internal data stream error`). jpegdec 는 200프레임 2.5초로 잘 돈다.
#
# ⚠ MJPG 는 해상도마다 fps 가 고정이다 (1280x720·1920x1080 = 100, 640x480 = 120).
#   다른 값을 주면 v4l2src 가 not-negotiated 로 즉사한다. videorate 로 낮춘다.
MJPEG_FPS=${NAVI_MJPEG_FPS:-100}
case "${NAVI_INPUT:-uyvy}" in
    mjpeg) SRC_CAPS="image/jpeg,width=$W,height=$H,framerate=$MJPEG_FPS/1"
           DECODE="jpegdec" ;;
    *)     SRC_CAPS="video/x-raw,format=UYVY,width=$W,height=$H,framerate=$IN_FPS/1"
           DECODE="identity" ;;
esac

start_pipeline() {
    gst-launch-1.0 -q \
        v4l2src device="$DEV" io-mode=mmap \
        ! $SRC_CAPS \
        ! queue max-size-buffers=4 leaky=downstream \
        ! $DECODE \
        ! videorate drop-only=true ! video/x-raw,framerate=$FPS/1 \
        ! videoconvert ! video/x-raw,format=$FMT \
        ! queue max-size-buffers=2 leaky=downstream \
        ! $ENC \
        ! h264parse config-interval=-1 \
        ! queue max-size-buffers=8 leaky=downstream \
        ! rtspclientsink location="rtsp://127.0.0.1:8554/$PATHNAME" latency=0 protocols=tcp \
        > "$LOG" 2>&1 &
    echo $!
}
# ⚠ 파이프(`| head -c ...`)로 로그 상한을 걸지 말 것.
#   `cmd | { ... } &` 에서 $! 는 파이프라인의 **마지막** 프로세스(서브셸)를 가리킨다.
#   그러면 워치독이 gst 가 아닌 엉뚱한 프로세스의 CPU 를 보게 되어 영원히 안 늘고
#   (혹은 영원히 멈춘 것처럼 보여) 감지가 통째로 무력해진다.
#   로그 폭주 방어는 아래 워치독 루프에서 파일 크기를 보고 처리한다.

# 🔴 워치독 — 카메라가 조용히 죽는다 (2026-07-31)
#
#    USB 가 `uvcvideo ... Non-zero status (-71)` 로 끊기면 카메라가 프레임 공급을
#    멈추는데, v4l2src 는 에러를 내지 않고 poll() 에서 영원히 기다린다.
#    그래서 겉보기엔 완전히 정상이다 — 프로세스 살아 있고, 로그 깨끗하고,
#    mediamtx 는 "is publishing" 이라고 표시한다. 실제로는 0바이트가 흐른다.
#
#    멈춘 프로세스의 스레드를 보면 이렇게 갈린다:
#        v4l2src0:src     do_sys_poll        ← 혼자 카메라를 기다린다
#        queue*/mpph264enc futex_wait_queue   ← 나머지는 할 일이 없어 대기
#
#    -71 은 몇 분 간격으로 산발적으로 난다(전기적 문제로 의심). 소프트웨어로는 못 막으니
#    멈춘 걸 감지해서 다시 세운다. 로봇은 현장에서 사람이 못 고친다.
#
#    감지 지표는 프로세스의 CPU 시간이다. 인코딩 중이면 계속 늘고, 멈추면 안 는다.
#    "프로세스가 살아있는가"는 지표가 못 된다 — 멈춰도 살아 있다.
WD_PERIOD=${NAVI_WD_PERIOD:-10}       # 몇 초마다 확인할지
WD_GRACE=${NAVI_WD_GRACE:-3}          # 이만큼 연속으로 기준 미달이면 죽은 것으로 본다
WD_MIN_TICK=${NAVI_WD_MIN_TICK:-20}   # 주기당 최소 CPU tick (100tick=1초). 이 미만이면 정지로 본다
WD_MAX=${NAVI_WD_MAX:-0}              # 재시작 횟수 상한 (0 = 무제한)

restarts=0
while : ; do
    GP=$(start_pipeline)
    echo "cam_stream: 파이프라인 시작 (pid=$GP, 재시작 $restarts회)" >&2

    prev=""; stuck=0
    while kill -0 "$GP" 2>/dev/null; do
        sleep "$WD_PERIOD"
        # utime+stime. comm 에 공백/괄호가 섞일 수 있으므로 ')' 뒤부터 센다
        # (전체 기준 14,15번째 = ')' 뒤 기준 12,13번째)
        cur=$(awk -F')' '{split($NF,f," "); print f[12]+f[13]}' "/proc/$GP/stat" 2>/dev/null)
        [ -z "$cur" ] && break                    # 프로세스가 사라졌다

        # 로그 폭주 방어 — mpph264enc 는 설정이 어긋나면 같은 에러를 초당 수만 줄 쏟는다.
        # 그게 /tmp(tmpfs)를 채우면 RAM 을 다 먹어 유저스페이스가 마비된다.
        SZ=$(wc -c < "$LOG" 2>/dev/null || echo 0)
        if [ "$SZ" -ge "$LOG_MAX_BYTES" ]; then
            echo "cam_stream: ⚠ 로그 폭주 (${SZ}B) — 파이프라인을 죽인다. 앞부분:" >&2
            head -3 "$LOG" >&2
            kill -9 "$GP" 2>/dev/null
            break
        fi
        # ⚠ "직전과 완전히 같은가"로 판정하면 안 된다.
        #    멈춰도 tick 이 주기당 2~3 씩 찔끔 는다(감시 스레드·타이머가 도는 몫).
        #    그러면 매번 값이 달라 stuck 이 0 으로 리셋되고 워치독이 영영 안 뜬다.
        #    실제로 이 조건 때문에 3분 39초를 멈춘 채로 방치했다.
        #    정상 인코딩 중이면 10초에 수백 tick 을 쓴다 — 넉넉히 잡아도 구분이 된다.
        delta=$((${cur:-0} - ${prev:-0}))
        if [ -n "$prev" ] && [ "$delta" -lt "$WD_MIN_TICK" ]; then
            stuck=$((stuck + 1))
            echo "cam_stream: ⚠ 정지 감지 ${stuck}/${WD_GRACE} (${WD_PERIOD}초간 ${delta}tick — 카메라가 프레임을 안 준다)" >&2
            if [ "$stuck" -ge "$WD_GRACE" ]; then
                echo "cam_stream: 파이프라인 재시작" >&2
                kill "$GP" 2>/dev/null
                sleep 3
                kill -9 "$GP" 2>/dev/null
                break
            fi
        else
            stuck=0
        fi
        prev=$cur
    done

    wait "$GP" 2>/dev/null
    restarts=$((restarts + 1))
    [ "$WD_MAX" -gt 0 ] && [ "$restarts" -ge "$WD_MAX" ] && {
        echo "cam_stream: 재시작 상한 $WD_MAX 회 도달 — 종료" >&2
        exit 1
    }
    # 강제 종료 직후 곧바로 열면 TRY_FMT 가 I/O error 를 낸다. 장치가 풀릴 시간을 준다.
    sleep 5
done
