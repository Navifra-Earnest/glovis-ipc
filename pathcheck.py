#!/usr/bin/env python3
"""유선·무선 **양쪽 경로가 다 되는지** 한 번에 확인한다.

만든 이유: 2026-08-18 에 무선으로 돌리다가 리셋 버튼이 안 먹었다. 원인은 ssh 명령에
유선 IP 가 박혀 있던 것이었는데, **어디에도 실패가 안 보였다**(journal 만 봤어야 함).
"한 경로에서만 되는 기능"은 이렇게 조용히 생긴다 → 경로별로 전부 찍어 본다.

  python3 pathcheck.py              # 양쪽 다 검사
  python3 pathcheck.py --hosts 192.168.50.1
  python3 pathcheck.py --selftest   # 판정 로직 검증 (하드웨어 불필요)

검사 항목(경로마다):
  1883  MQTT      — 명령·상태·열화상 프레임이 다니는 길
  5000  영상       — IR H.264 TCP
  22    ssh       — **리셋 버튼**이 `systemctl restart navi` 를 실행하는 길
  ssh키            — BatchMode 무인 접속. 이게 막히면 리셋 버튼이 조용히 죽는다
  state            — 실제로 데이터가 오는지 (포트 열림 ≠ 동작)
"""
import argparse
import json
import subprocess
import sys
import time

import mqtt_link

PORTS = (("MQTT", 1883), ("영상", 5000), ("ssh", 22))


def ssh_key_ok(host, user="radxa", timeout=8):
    """무인(BatchMode) ssh 가 되는지. 리셋 버튼이 쓰는 것과 같은 옵션으로 본다."""
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
           "-o", "StrictHostKeyChecking=accept-new", f"{user}@{host}", "true"]
    try:
        r = subprocess.run(cmd, capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, "타임아웃"
    if r.returncode == 0:
        return True, ""
    err = r.stderr.decode(errors="replace").strip().splitlines()
    return False, err[-1] if err else f"rc={r.returncode}"


def mqtt_probe(host, port, prefix, secs):
    """secs 동안 받은 것: state 개수 · 열화상 프레임 개수 · 마지막 state 요약.

    발행은 하지 않는다 — 점검이 로봇 상태를 바꾸면 안 된다.
    (열화상 프레임은 콘솔이 `cmd/stream` 으로 이미 켜 둔다. 콘솔이 안 떠 있으면 0 이 정상)
    """
    import paho.mqtt.client as mqtt

    got = {"state": 0, "frame": 0, "online": None, "last": None, "err": None}

    def on_connect(c, _u, _f, rc):
        c.subscribe([(f"{prefix}/state", 0), (f"{prefix}/frame/thermal", 0),
                     (f"{prefix}/state/online", 1)])

    def on_message(_c, _u, msg):
        if msg.topic.endswith("/frame/thermal"):
            got["frame"] += 1
        elif msg.topic.endswith("/state/online"):
            try:
                got["online"] = json.loads(msg.payload).get("online")
            except ValueError:
                pass
        else:
            got["state"] += 1
            try:
                got["last"] = json.loads(msg.payload)
            except ValueError:
                pass

    c = mqtt.Client()
    c.on_connect, c.on_message = on_connect, on_message
    try:
        c.connect(host, port, 10)
    except OSError as e:
        got["err"] = str(e)
        return got
    c.loop_start()
    time.sleep(secs)
    c.loop_stop()
    c.disconnect()
    return got


def verdict(row):
    """경로 하나의 판정 → (기호, 사유). 순수 함수라 selftest 로 검증한다.

    포트가 열려도 데이터가 안 오면 정상이 아니다 — 좀비 연결·navi 정지가 그렇다.
    """
    if not row["MQTT"]:
        return "❌", "경로 자체가 안 닿는다 (케이블/AP 확인)"
    bad = []
    if not row["영상"]:
        bad.append("영상 포트(5000) 닫힘 — navi 정지?")
    if not row["ssh"]:
        bad.append("ssh 포트(22) 닫힘")
    elif not row["ssh키"]:
        bad.append("ssh 키 인증 실패 — **리셋 버튼이 이 경로에서 안 먹는다**")
    if row["state"] == 0:
        bad.append("state 수신 0 — 포트만 열려 있다")
    return ("✅", "전 기능 사용 가능") if not bad else ("⚠️", " · ".join(bad))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hosts", default=",".join(mqtt_link.HOSTS),
                    help="검사할 경로. **앞이 우선순위**다 (유선 → 무선)")
    ap.add_argument("--prefix", default="navi")
    ap.add_argument("--secs", type=float, default=4.0, help="MQTT 수신 관찰 시간")
    ap.add_argument("--user", default="radxa", help="로봇 ssh 계정")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    hosts = a.hosts.split(",")
    rows, ok_all = [], True
    for i, h in enumerate(hosts):
        label = "유선(우선)" if i == 0 else "무선(폴백)"
        row = {"host": h, "label": label}
        for name, port in PORTS:
            row[name] = mqtt_link.reachable(h, port, 1.0)
        row["ssh키"], why = ssh_key_ok(h, a.user) if row["ssh"] else (False, "포트 닫힘")
        m = mqtt_probe(h, 1883, a.prefix, a.secs) if row["MQTT"] else {
            "state": 0, "frame": 0, "online": None, "last": None}
        row.update(state=m["state"], frame=m["frame"], online=m["online"],
                   last=m["last"], ssh_why=why)
        rows.append(row)

    print(f"\n경로 점검 — {a.secs:.0f}초 관찰\n")
    print(f"  {'경로':<12}{'주소':<16}{'1883':>6}{'5000':>6}{'22':>5}"
          f"{'ssh키':>7}{'state':>7}{'열화상':>8}")
    for r in rows:
        y = lambda v: "  ○" if v else "  ✗"          # noqa: E731 — 표 한 줄용
        print(f"  {r['label']:<12}{r['host']:<16}{y(r['MQTT']):>6}{y(r['영상']):>6}"
              f"{y(r['ssh']):>5}{y(r['ssh키']):>7}{r['state']:>7}{r['frame']:>8}")
    print()
    for r in rows:
        mark, why = verdict(r)
        print(f"  {mark} {r['label']} {r['host']}: {why}")
        if not r["ssh키"] and r["ssh"]:
            print(f"       └ {r['ssh_why']}")
            print(f"       └ 조치: ssh-copy-id {a.user}@{r['host']}")
        ok_all = ok_all and mark == "✅"

    live = [r for r in rows if r["MQTT"]]
    if live and live[0] is not rows[0]:
        print(f"\n  ℹ️ 유선({rows[0]['host']})이 안 닿아 무선으로 동작 중이다."
              f" 케이블을 꽂으면 자동으로 유선으로 돌아온다(즉시 승격).")
    if live and live[0]["last"]:
        d = live[0]["last"]
        print(f"\n  로봇: estop={d.get('estop')}  drive_ok={d.get('drive_ok')}"
              f"  kinematics={d.get('kinematics')}  online={live[0]['online']}")
    if not live:
        print("\n  ❌ 어느 경로도 안 닿는다 — 로봇 전원/AP/케이블부터 확인한다.")
    return 0 if ok_all else 1


def selftest():
    base = {"MQTT": True, "영상": True, "ssh": True, "ssh키": True, "state": 4}
    assert verdict(base)[0] == "✅"
    assert verdict({**base, "MQTT": False})[0] == "❌"
    # 🔴 이게 이번 버그다 — 포트는 열려 있는데 키 인증이 안 돼 리셋만 죽는 경우
    m, why = verdict({**base, "ssh키": False})
    assert m == "⚠️" and "리셋" in why
    # 포트만 열리고 데이터가 안 오는 경우(좀비 연결·navi 정지)도 정상이 아니다
    assert verdict({**base, "state": 0})[0] == "⚠️"
    assert verdict({**base, "영상": False})[0] == "⚠️"
    print("selftest OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
