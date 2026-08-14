#!/usr/bin/env python3
"""누리로봇 SB60 구동 테스트 — 가감속 속도제어(0x03)로 정/역 회전.

감속비 16이 설정돼 있으면 속도·위치는 모두 **출력축 기준**이다.
어떤 식으로 끝나든(Ctrl+C 포함) 반드시 정지 명령을 보내고 나간다.

  python3 nuri_spin.py                    # 출력축 10RPM, 각 방향 2초
  python3 nuri_spin.py --rpm 5 --hold 1
"""
import argparse
import statistics
import struct
import sys
import time

from nuri_ping import Bus, frame

MODE_ACC_SPEED = 0x03
REQ_POS, FEED_POS = 0xA1, 0xD1
# 전류 피드백은 평균이 아니라 순간(피크) 샘플이다. 무부하 10RPM에서도 0.0~8.7A로 튀고
# 0xFE(25.4A) 포화가 수시로 나온다. 그래서 정격(2.7A) 기준 감시는 의미가 없고,
# 스톨(14.8A)에 가까운 값이 중앙값으로 지속될 때만 자른다. 실측으로 다시 잡을 것.
# ponytail: 임계는 경험값, 스톨 검출을 제대로 하려면 부하 걸고 분포부터 재야 한다
CUR_TRIP_DEFAULT = 12.0


def speed(dev_id, ccw, rpm, ramp_s):
    """가감속 속도제어: 방향(1) 속도(2, 0.1RPM) 도달시간(1, 0.1s)"""
    return frame(dev_id, MODE_ACC_SPEED,
                 bytes([0x00 if ccw else 0x01])
                 + struct.pack(">H", max(0, min(0xFFFD, int(round(rpm * 10)))))
                 + bytes([max(1, min(255, int(round(ramp_s * 10))))]))


def read_pos(bus, dev_id):
    got, err = bus.xfer(frame(dev_id, REQ_POS), FEED_POS)
    if not got:
        return None, err
    v = got[2]
    pos, spd = struct.unpack(">HH", v[1:5])
    return ("CW" if v[0] else "CCW", pos / 100, spd / 10, v[5] / 10, v.hex(" ")), None


def monitor(bus, dev_id, seconds, period=0.15):
    """구동 없이 피드백만 관측. 전류 필드가 실제 값인지 확인용 — raw를 그대로 찍는다."""
    print(f"피드백 관측 {seconds}s (구동 명령 없음)\n")
    t0 = time.monotonic()
    hist = []
    while (t := time.monotonic() - t0) < seconds:
        time.sleep(period)
        got, err = read_pos(bus, dev_id)
        if not got:
            print(f"  {t:4.1f}s  실패: {err}")
            continue
        d, pos, spd, cur, raw = got
        hist.append(cur)
        print(f"  {t:4.1f}s  {d:3}  {pos:7.2f}°  {spd:6.1f} RPM  {cur:5.1f} A   raw={raw}")
    if hist:
        print(f"\n  전류: 최소 {min(hist):.1f} / 최대 {max(hist):.1f} / 평균 {sum(hist)/len(hist):.1f} A"
              f"   0xFE(25.4A) 발생 {sum(1 for c in hist if c >= 25.4)}/{len(hist)}회")


def run_leg(bus, dev_id, ccw, rpm, ramp, hold, trip=CUR_TRIP_DEFAULT):
    label = "CCW(정)" if ccw else "CW(역)"
    print(f"\n[{label}] {rpm} RPM, 가감속 {ramp}s, {hold}s 유지")
    bus.send(speed(dev_id, ccw, rpm, ramp))
    t0 = time.monotonic()
    peak, window = 0.0, []
    while (t := time.monotonic() - t0) < hold:
        time.sleep(0.15)
        got, err = read_pos(bus, dev_id)
        if not got:
            print(f"  {t:4.1f}s  피드백 실패: {err}")
            continue
        d, pos, spd, cur, raw = got
        peak = max(peak, cur)
        # 전류 피드백은 순간 샘플이라 0xFE(25.4A) 포화가 수시로 튄다.
        # 중앙값으로 스파이크를 걸러내고 판단한다.
        window = (window + [cur])[-5:]
        med = statistics.median(window)
        print(f"  {t:4.1f}s  {d:3}  {pos:7.2f}°  {spd:6.1f} RPM  {cur:5.1f} A (중앙 {med:4.1f})"
              + (f"  raw={raw}" if cur >= 25.4 else ""))
        if len(window) == 5 and med > trip:
            print(f"  ⛔ 전류 중앙값 {med:.1f}A > {trip}A — 중단 (부하 확인 필요)")
            return peak, False
    return peak, True


def stop(bus, dev_id, ramp=0.5):
    """감속 정지 후 전류가 실제로 0인지 확인한다.

    속도 0 명령만으로 안 떨어지는 경우가 실측에서 나왔다(정지 상태로 6.5A 지속).
    그때는 Open-loop 듀티 0으로 출력을 끊는다 — 문서 5.4 "상태 유지를 위한 동작이 없다".
    제어 Off(0x0A)는 EEPROM에 쓰이므로 최후 수단으로만 쓴다.
    """
    bus.send(speed(dev_id, True, 0, ramp))
    time.sleep(ramp + 0.5)
    got, _ = read_pos(bus, dev_id)
    if got and got[3] > 0.5:
        print(f"  정지했는데 전류 {got[3]:.1f}A — Open-loop 듀티 0으로 출력 차단")
        bus.send(frame(dev_id, 0x11, b"\x00\x00\x00"))
        time.sleep(0.3)
        got, _ = read_pos(bus, dev_id)
        if got and got[3] > 0.5:
            print(f"  여전히 {got[3]:.1f}A — 제어 Off")
            bus.send(frame(dev_id, 0x0A, b"\x01"))
            time.sleep(0.4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS2")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--id", type=lambda x: int(x, 0), default=0)
    ap.add_argument("--guard", type=float, default=200.0)
    ap.add_argument("--rpm", type=float, default=10.0, help="출력축 속도 [RPM]")
    ap.add_argument("--ramp", type=float, default=1.0, help="가감속 도달시간 [s]")
    ap.add_argument("--hold", type=float, default=2.0, help="방향별 유지 시간 [s]")
    ap.add_argument("--trip", type=float, default=CUR_TRIP_DEFAULT, help="전류 중앙값 트립 [A]")
    ap.add_argument("--monitor", type=float, metavar="SEC",
                    help="구동 없이 피드백만 관측 (전류 필드 검증용)")
    a = ap.parse_args()

    bus = Bus(a.port, a.baud, guard_us=a.guard)
    print(f"{a.port} @ {a.baud} bps, ID {a.id}  (감속비 16 가정 → 출력축 기준)")

    if a.monitor:
        try:
            monitor(bus, a.id, a.monitor)
        finally:
            bus.close()
        return

    got, err = read_pos(bus, a.id)
    if not got:
        bus.close()
        sys.exit(f"시작 전 피드백 실패: {err}")
    print(f"시작 위치: {got[1]:.2f}°")

    ok = True
    try:
        for ccw in (True, False):
            peak, ok = run_leg(bus, a.id, ccw, a.rpm, a.ramp, a.hold, a.trip)
            stop(bus, a.id)
            got, _ = read_pos(bus, a.id)
            print(f"  정지 → {got[1]:.2f}°  {got[2]:.1f} RPM   (최대 전류 {peak:.1f} A)"
                  if got else "  정지 (피드백 실패)")
            if not ok:
                break
    except KeyboardInterrupt:
        print("\n중단됨")
        ok = False
    finally:
        stop(bus, a.id)
        time.sleep(0.5)
        bus.close()

    print("\n완료" if ok else "\n비정상 종료 — 정지 명령은 보냈습니다")


if __name__ == "__main__":
    main()
