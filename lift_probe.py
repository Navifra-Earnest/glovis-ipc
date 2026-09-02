#!/usr/bin/env python3
"""리프트(액추에이터) 전류 실측 — 무부하 상승 vs 차체 접촉 전류를 가른다.

왜 필요한가: 리프트가 차체 하부에 닿은 뒤에도 계속 밀면 모터가 상한다. 전류로
막힘을 판정해 상승 명령을 끊으려면 **무부하 전류와 접촉 전류의 실측 간격**이 있어야
임계값을 정할 수 있다. 문서에는 정격(MA5 2 A)만 있고 실제 동작 전류는 없다.

  python3 lift_probe.py                 # 60초 관찰 후 요약
  python3 lift_probe.py --secs 30
  python3 lift_probe.py --selftest      # 판정 로직 검증 (하드웨어 불필요)

⚠️ 발행은 하지 않는다. 리프트는 **사용자가 물리버튼으로** 올리고 내린다.

읽는 값은 `state.actuator` 의 `current`(EMA 평활, 계수 0.3) · `position`(홀 카운트) ·
`state`(정지/구동중/목표도달/스트로크끝단/막힘/FAULT).

🔴 `stateDigest` 에 **전류가 없다**(mqtt.hpp:291). 그래서 리프트가 움직이는 동안은
   position 변화로 5 Hz 가 오지만, **멈추면 2초 heartbeat 로 떨어진다.** 접촉 순간의
   전류 상승은 그 사이에 묻힐 수 있다 → 아래 요약은 "정지 구간" 을 따로 집계한다.
"""
import argparse
import json
import sys
import time

import mqtt_link

MOVING_MIN_DELTA = 2        # 홀 카운트가 이보다 변하면 "움직이는 중" 으로 본다


def classify(samples):
    """[(t, pos, cur, st)] → (움직임 구간, 정지 구간). 위치 변화로 가른다.

    전류만으로는 못 가른다 — 무부하 상승과 접촉 초기 전류가 겹칠 수 있다.
    **위치가 변하면 실제로 움직이는 것**이고, 멈췄는데 전류가 흐르면 밀고 있는 것이다.
    """
    moving, stalled = [], []
    for (t0, p0, c0, _s0), (t1, p1, c1, s1) in zip(samples, samples[1:]):
        (moving if abs(p1 - p0) >= MOVING_MIN_DELTA else stalled).append((t1, c1, s1))
    return moving, stalled


def stats(rows):
    """[(t, cur, st)] → (개수, 중앙, 최대)."""
    if not rows:
        return 0, None, None
    c = sorted(r[1] for r in rows)
    return len(c), c[len(c) // 2], c[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hosts", default=",".join(mqtt_link.HOSTS))
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--prefix", default="navi")
    ap.add_argument("--secs", type=float, default=60.0)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    import paho.mqtt.client as mqtt
    host = mqtt_link.pick(a.hosts.split(","), a.port)
    if not host:
        print("❌ 브로커에 안 닿는다")
        return 1
    print(f"브로커 {host} · {a.secs:.0f}초 관찰. 리프트를 물리버튼으로 조작할 것.\n")
    print(f"  {'시각':>7}  {'위치':>8}{'전류A':>7}  상태")

    samples, seen_states = [], []
    t0 = time.monotonic()

    def on_connect(c, _u, _f, _rc):
        c.subscribe(a.prefix + "/state", 0)

    def on_message(_c, _u, msg):
        try:
            act = (json.loads(msg.payload).get("actuator") or {})
        except ValueError:
            return
        if not act.get("present"):
            return
        t = time.monotonic() - t0
        pos, cur, st = act.get("position"), act.get("current"), act.get("state")
        if pos is None or cur is None:
            return
        samples.append((t, pos, cur, st))
        if not seen_states or seen_states[-1][1] != st:
            seen_states.append((t, st))
        print(f"  {t:6.1f}s  {pos:>8}{cur:>7.2f}  {st}", flush=True)

    c = mqtt.Client()
    c.on_connect, c.on_message = on_connect, on_message
    c.connect(host, a.port, 10)
    c.loop_start()
    time.sleep(a.secs)
    c.loop_stop()
    c.disconnect()

    moving, stalled = classify(samples)
    print(f"\n── 요약 (샘플 {len(samples)}개)\n")
    for label, rows in (("움직이는 중 (위치 변화)", moving), ("정지 (위치 고정)", stalled)):
        n, med, mx = stats(rows)
        if n:
            print(f"  {label:<22} {n:>4}개   중앙 {med:.2f} A   최대 {mx:.2f} A")
        else:
            print(f"  {label:<22}    0개")
    print("\n  상태 변화:", " → ".join(f"{t:.1f}s {s}" for t, s in seen_states) or "없음")

    _, med_mv, _ = stats(moving)
    _, _, max_st = stats(stalled)
    if med_mv is not None and max_st is not None and max_st > med_mv:
        # 무부하 위쪽과 접촉 아래쪽 사이를 잡는다. 여유를 위해 중간보다 살짝 위.
        print(f"\n  → 무부하 중앙 {med_mv:.2f} A · 정지 최대 {max_st:.2f} A"
              f"  ⇒ 임계 후보 **{med_mv + (max_st - med_mv) * 0.6:.2f} A**")
    else:
        print("\n  → 두 구간이 안 갈렸다. 무부하 상승과 접촉을 각각 확실히 만들어 다시 잰다.")
    return 0


def selftest():
    # 위치가 변하면 움직임, 고정이면 정지로 갈라야 한다
    s = [(0.0, 100, 0.4, "구동중"), (0.2, 110, 0.5, "구동중"), (0.4, 120, 0.5, "구동중"),
         (0.6, 120, 1.2, "구동중"), (0.8, 120, 1.9, "막힘")]
    mv, st = classify(s)
    assert len(mv) == 2 and len(st) == 2, (mv, st)
    assert [r[1] for r in mv] == [0.5, 0.5]
    assert [r[1] for r in st] == [1.2, 1.9]
    # 홀 카운트 1 변화는 노이즈로 본다 (MOVING_MIN_DELTA)
    assert classify([(0.0, 100, 0.4, "x"), (0.2, 101, 0.4, "x")])[1], "1카운트는 정지"
    n, med, mx = stats([(0, 0.5, "x"), (1, 0.7, "x"), (2, 0.6, "x")])
    assert (n, med, mx) == (3, 0.6, 0.7)
    assert stats([]) == (0, None, None)
    print("selftest OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
