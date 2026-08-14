#!/usr/bin/env python3
"""속도/위치 제어기 파라미터 조회 (Kp Ki Kd 정격전류).

만든 이유: 명령 20 RPM 인데 실측이 11.4 RPM 에서 포화했다.
5·10 RPM 은 명령대로 정확히 나오므로 소프트웨어 변환 문제가 아니고,
전류가 0xFE(25.4A) 로 계속 포화하는 걸 보면 모터가 토크 한계에 걸린 것이다.
그 한계를 정하는 게 **정격전류 설정**이라 값을 직접 봐야 한다.

  0xA4 → 0xD4  속도제어기  Kp Ki Kd 정격전류
  0xA3 → 0xD3  위치제어기  Kp Ki Kd 정격전류
  정격전류 단위 100mA (예: 3.2A → 0x20)

  python3 nuri_ctlparam.py --id 3 --baud 115200
"""
import argparse
import sys

sys.path.insert(0, "/home/radxa/navi_src")
from nuri_ping import Bus, frame, DE_CHIP, DE_LINE   # noqa: E402

REQ = {
    "속도제어기": (0xA4, 0xD4),
    "위치제어기": (0xA3, 0xD3),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS2")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--id", type=lambda x: int(x, 0), default=3)
    ap.add_argument("--guard", type=float, default=200.0)
    a = ap.parse_args()

    print(f"DE={DE_CHIP}:{DE_LINE}  ID={a.id} @{a.baud}")
    bus = Bus(a.port, a.baud, guard_us=a.guard)
    try:
        for name, (req, resp) in REQ.items():
            got, err = bus.xfer(frame(a.id, req), resp)
            if not got:
                print(f"  {name}: {err}")
                continue
            _, _, v = got
            if len(v) < 4:
                print(f"  {name}: 값이 짧다 raw={v.hex(' ')}")
                continue
            kp, ki, kd, cur = v[0], v[1], v[2], v[3]
            print(f"  {name}: Kp={kp} Ki={ki} Kd={kd}  정격전류={cur * 0.1:.1f}A (0x{cur:02X})")
    finally:
        bus.close()

    print("\n정격전류가 낮으면 고속에서 토크가 모자라 속도가 포화한다.")
    print("SB60PB60RNB 사양과 대조해 볼 것 — 낮으면 0x05(속도제어기 설정)로 올린다.")
    print("⚠ 제어 루프가 도는 중에 설정을 바꾸면 안 된다(문서: 오작동·과전류).")


if __name__ == "__main__":
    main()
