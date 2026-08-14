#!/usr/bin/env python3
"""버튼 ↔ 입력비트 매핑 (읽기 전용). 누른 **순서**로 가른다.

시간창 방식은 원격 실행 지연 때문에 구간이 밀려 섞였다. 그래서 시각이 아니라
**상승엣지가 나타난 순서**로 판정한다 — 사용자는 순서만 지키면 되고 속도는 자유다.

  python3 crevis_map.py                 # 전진 → 후진 → 리셋 순서로 눌렀다 떼기
"""
import argparse
import time

from crevis_probe import Modbus

LABELS = ("전진", "후진", "리셋")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.100.100")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=1)
    ap.add_argument("--bits", type=int, default=16)
    ap.add_argument("--seconds", type=float, default=40.0)
    a = ap.parse_args()

    m = Modbus(a.host, a.port, a.unit)
    prev = [0] * a.bits
    order = []          # 처음 눌린 순서대로 비트번호
    press = {}          # 비트별 누른 횟수
    t0 = time.time()

    while time.time() - t0 < a.seconds:
        cur = m.read_bits(2, 0x0000, a.bits)
        for i, (p, c) in enumerate(zip(prev, cur)):
            if c and not p:                       # 상승엣지
                press[i] = press.get(i, 0) + 1
                if i not in order:
                    order.append(i)
                print(f"  [{time.time() - t0:5.1f}s] bit{i} 눌림"
                      f"{'  ← 처음' if press[i] == 1 else f'  ({press[i]}번째)'}", flush=True)
        prev = cur
        if len(order) >= len(LABELS):
            time.sleep(1.0)                        # 마지막 누름 마무리 대기
            break
        time.sleep(0.02)

    print("\n=== 매핑 결과 (누른 순서 기준) ===")
    if len(order) < len(LABELS):
        print(f"  ⚠️ {len(order)}개만 감지됐다: {['bit%d' % b for b in order]}")
        print("     3개 다 눌렀는데 이러면 미감지 버튼의 배선·접점을 봐야 한다")
    for lab, bit in zip(LABELS, order):
        print(f"  {lab:4} → **bit{bit}**   (누른 횟수 {press.get(bit, 0)})")
    if len(order) > len(LABELS):
        print(f"  그 외 감지: {['bit%d' % b for b in order[len(LABELS):]]}")


if __name__ == "__main__":
    main()
