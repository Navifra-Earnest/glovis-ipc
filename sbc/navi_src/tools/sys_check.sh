#!/bin/bash
# sys_check.sh — 보드 전체 세팅 점검. 문제 될 만한 것을 한 번에 뽑는다.
#
#   sudo sh sys_check.sh
#
# 원격 조작용 로봇에 올릴 보드라 "죽지 않는 것"이 최우선이다.
# 그래서 로그·전원·온도·네트워크 안정성 위주로 본다.
echo "════════════════════════════════════════ 시스템"
tr -d '\0' < /sys/firmware/devicetree/base/model; echo
echo "커널   : $(uname -r)"
echo "OS     : $(. /etc/os-release; echo "$PRETTY_NAME")"
echo "가동   : $(uptime -p 2>/dev/null || uptime)"
echo "부팅   : $(who -b 2>/dev/null | awk '{print $3, $4}')"
echo "cmdline: $(cat /proc/cmdline)"

echo
echo "════════════════════════════════════════ 전원·온도 (죽는 원인 1순위)"
for z in /sys/class/thermal/thermal_zone*; do
    [ -r "$z/temp" ] && printf "  %-12s %s°C\n" "$(cat $z/type 2>/dev/null)" "$(( $(cat $z/temp) / 1000 ))"
done
echo "  CPU 주파수:"
for p in /sys/devices/system/cpu/cpufreq/policy*; do
    [ -d "$p" ] && printf "    %s: %s MHz (거버너 %s, 최대 %s MHz)\n" "$(basename $p)" \
        "$(( $(cat $p/scaling_cur_freq) / 1000 ))" "$(cat $p/scaling_governor)" \
        "$(( $(cat $p/scaling_max_freq) / 1000 ))"
done
echo "  스로틀링 이력:"
grep -icE "thermal|throttl|over-current|under-voltage" /var/log/kern.log 2>/dev/null \
    | sed 's/^/    kern.log 관련 줄 수: /' || echo "    (로그 없음)"

echo
echo "════════════════════════════════════════ 메모리·스토리지"
free -h | head -2
echo "  OOM 발생 이력: $(dmesg 2>/dev/null | grep -ci "out of memory\|oom-killer" || echo '조회불가')"
df -h / /tmp 2>/dev/null | grep -vE "^Filesystem" | awk '{printf "  %-12s %s / %s (%s)\n", $6, $3, $2, $5}'
echo "  루트 파일시스템: $(findmnt -no SOURCE,FSTYPE,OPTIONS / | cut -c1-90)"

echo
echo "════════════════════════════════════════ 네트워크 (이더넷 안정성)"
ip -br link | grep -vE "^lo"
for i in $(ls /sys/class/net | grep -vE "^lo"); do
    d=$(basename $(readlink -f /sys/class/net/$i/device/driver) 2>/dev/null)
    echo "  [$i] 드라이버=$d 속도=$(cat /sys/class/net/$i/speed 2>/dev/null||echo -)Mbps"
    # 에러 카운터가 0이 아니면 물리/드라이버 문제를 의심한다
    for c in rx_errors tx_errors rx_dropped tx_dropped rx_crc_errors tx_carrier_errors; do
        v=$(cat /sys/class/net/$i/statistics/$c 2>/dev/null || echo 0)
        [ "$v" != "0" ] && echo "      ⚠ $c = $v"
    done
done
echo "  NETDEV WATCHDOG 이력: $(grep -ci "NETDEV WATCHDOG" /var/log/kern.log 2>/dev/null || echo '로그없음')"

echo
echo "════════════════════════════════════════ USB"
echo "  autosuspend: $(cat /sys/module/usbcore/parameters/autosuspend)  (-1이어야 카메라가 안 끊긴다)"
lsusb | grep -viE "root hub" | sed 's/^/  /'
echo "  링크 속도:"
lsusb -t 2>/dev/null | grep -vE "Class=root_hub" | grep -oE "Class=[A-Za-z ]+, Driver=[a-z_-]+, [0-9]+M" | sed 's/^/    /'

echo
echo "════════════════════════════════════════ 로그 설정 (크래시 원인 추적용)"
echo "  rsyslog     : $(systemctl is-active rsyslog 2>/dev/null)"
echo "  kern.log    : $(ls -l /var/log/kern.log 2>/dev/null | awk '{print $5" bytes, "$6" "$7" "$8}' || echo '없음')"
echo "  journal 보존: $([ -d /var/log/journal ] && echo '예 (재부팅 후에도 남음)' || echo '아니오 (휘발)')"
echo "  journalctl -k: $(journalctl -k --no-pager 2>/dev/null | grep -cv "^--" || echo 0) 줄"
echo "  dmesg_restrict: $(sysctl -n kernel.dmesg_restrict 2>/dev/null)"

echo
echo "════════════════════════════════════════ 서비스 상태"
systemctl --failed --no-legend --no-pager 2>/dev/null | sed 's/^/  ⚠ /' || true
[ -z "$(systemctl --failed --no-legend --no-pager 2>/dev/null)" ] && echo "  실패한 서비스 없음"
echo "  시간 동기화: $(timedatectl show -p NTPSynchronized --value 2>/dev/null)  ($(date))"

echo
echo "════════════════════════════════════════ 우리 장치"
echo "  RS485 : $(ls /dev/ttyS2 2>/dev/null || echo '없음')   DE=gpiochip3 line3"
echo "  카메라: $(for d in /dev/video[0-9]*; do v4l2-ctl -d $d --info 2>/dev/null | grep -q "Card type" && echo -n "$d($(v4l2-ctl -d $d --info 2>/dev/null|grep 'Card type'|cut -d: -f2|tr -d ' ')) "; done)"
echo "  PWM   : $(ls /sys/class/pwm/ 2>/dev/null | tr '\n' ' ')"
echo "  VPU   : $(ls /dev/video-enc0 /dev/video-dec0 2>/dev/null | tr '\n' ' ')"
