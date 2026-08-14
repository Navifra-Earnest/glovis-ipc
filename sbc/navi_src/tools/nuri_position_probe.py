#!/usr/bin/env python3
"""위치 피드백의 의미를 판별한다. 위치 제어(moveTo)를 구현하기 전에 반드시 돌릴 것.

왜 필요한가 — 구동 로그에서 두 해석이 갈렸다:
  · 대부분의 leg는 명령 직후 위치가 0.19°로 점프했다 (쓰레기 첫 샘플?)
  · 그런데 한 번은 이전 값 125.07° → 125.28° 로 자연스럽게 이어졌다
  · CW 구동인데 위치는 9.89 → 146.50 으로 **증가**했다 (감소해야 할 것 같은데)

가려낼 것:
  (a) 명령 직후 첫 응답만 쓰레기고, 위치는 0점 기준 절대값으로 연속이다
  (b) 위치 = "0점에서 현재 방향으로 이동한 크기" + 방향 플래그 (문서 5.2.1)
      → 방향이 바뀌면 크기가 0을 지나 다시 증가한다
  (c) 명령을 보낼 때마다 위치 기준이 재설정된다

  python3 nuri_position_probe.py           # 30° 씩 CCW → 정지 → CW
  python3 nuri_position_probe.py --deg 90
"""
import argparse
import struct
import time

from nuri_ping import Bus, frame

REQ_POS, FEED_POS = 0xA1, 0xD1


def read(bus, dev_id):
    got, err = bus.xfer(frame(dev_id, REQ_POS), FEED_POS)
    if not got:
        return None
    v = got[2]
    pos, spd = struct.unpack(">HH", v[1:5])
    return ("CW" if v[0] else "CCW", pos / 100, spd / 10, v[5] / 10, v.hex(" "))


def show(bus, dev_id, tag):
    r = read(bus, dev_id)
    print(f"  {tag:22} " + ("실패" if not r else
          f"{r[0]:3} {r[1]:8.2f}°  {r[2]:6.1f} RPM  {r[3]:5.1f} A   raw={r[4]}"))
    return r


def leg(bus, dev_id, ccw, deg, rpm, label):
    """가감속 위치제어(0x02)로 목표 각도까지. 이동 중 전 샘플을 raw까지 찍는다."""
    print(f"\n{label}  목표 {deg}° {'CCW' if ccw else 'CW'}, 도달 2.0s")
    v = bytes([0x00 if ccw else 0x01]) + struct.pack(">H", int(deg * 100)) + bytes([20])
    bus.send(frame(dev_id, 0x02, v))
    t0 = time.monotonic()
    while (t := time.monotonic() - t0) < 4.0:
        time.sleep(0.2)
        show(bus, dev_id, f"{t:4.1f}s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS2")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--id", type=lambda x: int(x, 0), default=0)
    ap.add_argument("--guard", type=float, default=200.0)
    ap.add_argument("--deg", type=float, default=30.0, help="이동 각도 [출력축 도]")
    ap.add_argument("--rpm", type=float, default=5.0)
    a = ap.parse_args()

    bus = Bus(a.port, a.baud, guard_us=a.guard)
    print(f"{a.port} @ {a.baud} bps, ID {a.id}\n")

    print("① 제어 On (설정 명령이라 EEPROM 기록 — 400ms 대기)")
    bus.send(frame(a.id, 0x0A, b"\x00"))
    time.sleep(0.5)

    print("\n② 정지 상태에서 3회 — 명령 없이도 값이 흔들리는지")
    for i in range(3):
        show(bus, a.id, f"idle {i+1}")
        time.sleep(0.3)

    try:
        # ── 같은 방향으로 두 번: 위치가 누적되는지 (누적 = 절대 기준)
        leg(bus, a.id, True, a.deg, a.rpm, "③ CCW 1차")
        bus.send(frame(a.id, 0x11, b"\x00\x00\x00"))       # 정지
        time.sleep(0.5)
        print("\n   정지 후 3회 — 값이 유지되는지 (유지되면 첫 샘플만 쓰레기라는 뜻)")
        for i in range(3):
            show(bus, a.id, f"stopped {i+1}")
            time.sleep(0.3)

        leg(bus, a.id, True, a.deg * 2, a.rpm, "④ CCW 2차 (2배 각도)")
        bus.send(frame(a.id, 0x11, b"\x00\x00\x00"))
        time.sleep(0.5)
        before = show(bus, a.id, "반전 직전")

        # ── 방향 반전: 크기가 0을 지나 내려가는지, 아니면 0에서 다시 오르는지
        leg(bus, a.id, False, a.deg, a.rpm, "⑤ CW 반전")
        bus.send(frame(a.id, 0x11, b"\x00\x00\x00"))
        time.sleep(0.5)
        after = show(bus, a.id, "반전 후")

        print("\n── 판정 ─────────────────────────────────────────")
        if before and after:
            print(f"  반전 전 {before[0]} {before[1]:.2f}°  →  반전 후 {after[0]} {after[1]:.2f}°")
            print("  · 방향 플래그가 바뀌고 크기가 작아졌다      → (b) 0점 기준 크기+방향")
            print("  · 방향 그대로에 크기만 줄었다                → (a) 절대 위치, 부호 없음")
            print("  · 반전 후 값이 0 근처에서 다시 올라갔다      → (c) 명령마다 기준 재설정")
    finally:
        bus.send(frame(a.id, 0x11, b"\x00\x00\x00"))       # 듀티 0
        time.sleep(0.3)
        bus.send(frame(a.id, 0x0A, b"\x01"))               # 제어 Off (소음 방지)
        time.sleep(0.4)
        show(bus, a.id, "종료 (제어 Off)")
        bus.close()


if __name__ == "__main__":
    main()
