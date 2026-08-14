#!/usr/bin/env python3
"""ID 0 응답이 '단일 장치의 타이밍 문제'인지 '여러 장치의 충돌'인지 가른다 — 읽기 전용.

가설: 모터 여러 대가 공장기본값(ID 0)으로 같이 매달려 있다.
  → Ping 응답(0xD0)은 모든 장치가 **똑같은 바이트**를 내보내므로 겹쳐도 자주 해독된다.
  → 위치 피드백(0xD1)은 장치마다 위치·전류가 **달라서** 겹치면 거의 항상 깨진다.
단일 장치라면 두 응답의 성공률이 비슷해야 한다.

읽기 전용 명령(0xA0 Ping, 0xA1 위치요청)만 쓴다 — 모터는 움직이지 않는다.
"""
import sys
import time
from collections import Counter

sys.path.insert(0, "/home/radxa/navi_src/tools")
from nuri_ping import Bus, frame  # noqa: E402

N = 20
REQ_PING, FB_PING = 0xA0, 0xD0
REQ_POS, FB_POS = 0xA1, 0xD1


def trial(bus, req, fb):
    ok, raws = 0, Counter()
    for _ in range(N):
        got, err = bus.xfer(frame(0, req), fb)
        if got:
            ok += 1
            raws[bytes(got[2]).hex(" ")] += 1
        elif err:
            raws["ERR " + err.split("raw=")[-1]] += 1
        time.sleep(0.05)          # 문서 권장 10~50ms 이상 간격
    return ok, raws


bus = Bus("/dev/ttyS2", 9600, guard_us=200.0)
for name, req, fb in (("Ping    (모든 장치 응답 동일)", REQ_PING, FB_PING),
                      ("위치피드백 (장치마다 값 다름)", REQ_POS, FB_POS)):
    ok, raws = trial(bus, req, fb)
    print(f"\n{name}: {ok}/{N} 해독 성공")
    for v, n in raws.most_common(6):
        print(f"    {n:2d}x  {v}")
bus.close()
