#!/usr/bin/env python3
"""폴링 응답시간 실측 — 제어 명령 전후로 얼마나 달라지는지 본다.

navi 가 spin 중에 "휠 응답 없음"으로 e-stop 되는 문제를 가리기 위해 만들었다.
상태 모니터(폴링만)에서는 멀쩡한데 구동 명령을 보낸 뒤에만 실패했다.
"""
import argparse
import struct
import time

from nuri_ping import Bus, frame


def poll(bus, dev_id):
    t0 = time.perf_counter()
    got, err = bus.xfer(frame(dev_id, 0xA1), 0xD1)
    return (time.perf_counter() - t0) * 1000, got is not None, err


def show(bus, ids, n, tag):
    print(f"\n{tag}")
    worst = 0.0
    for k in range(n):
        i = ids[k % len(ids)]
        ms, ok, err = poll(bus, i)
        worst = max(worst, ms)
        print(f"   ID{i}  {ms:6.1f}ms  {'OK' if ok else err}")
        time.sleep(0.02)
    print(f"   → 최악 {worst:.1f}ms")
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS2")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--guard", type=float, default=300.0)
    ap.add_argument("--ids", default="2,3")
    ap.add_argument("--rpm", type=float, default=5.0)
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",")]

    # ⚠ pyserial 의 읽기 타임아웃이 크면 응답이 다 와도 나머지 바이트를 기다리느라
    #   측정값이 전부 그 타임아웃 값으로 찍힌다(0.3초로 두니 전부 302ms였다).
    #   실제 왕복은 115200에서 1~2ms 수준이라 짧게 잡아야 진짜 시간이 보인다.
    bus = Bus(a.port, a.baud, timeout=0.02, guard_us=a.guard)
    show(bus, ids, 6, "① 명령 없이 폴링")

    print("\n② 속도 명령 송신")
    for i in ids:
        v = bytes([0]) + struct.pack(">H", int(a.rpm * 10)) + bytes([10])
        bus.send(frame(i, 0x03, v))
        print(f"   ID{i} ← {a.rpm}RPM CCW")

    time.sleep(0.05)
    show(bus, ids, 8, "③ 명령 직후 폴링")
    time.sleep(1.0)
    show(bus, ids, 6, "④ 1초 후 폴링")

    for i in ids:
        bus.send(frame(i, 0x11, b"\x00\x00\x00"))   # Open-loop 듀티 0 = 정지
        time.sleep(0.05)
    print("\n정지 명령 전송 완료")
    bus.close()


if __name__ == "__main__":
    main()
